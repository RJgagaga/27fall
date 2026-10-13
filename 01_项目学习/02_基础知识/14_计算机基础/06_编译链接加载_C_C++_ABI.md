# 编译、链接、加载与 ABI：从源文件到运行中的进程，以及 C/C++ 语言机理

> **一句话定位**：`gcc main.c -o app` 看似一步，实际是预处理、编译、汇编、链接四个阶段，之后还有加载器把可执行文件放进内存。绝大多数让新手崩溃的错误，如 `undefined reference`、`multiple definition`、"运行时找不到 .so"、"升级库后程序崩溃"，都能在这条流水线上准确定位。本章用真实的 `gcc`/`nm`/`objdump` 输出带你一步步看：符号表长什么样、重定位怎么手算、静态库与动态库的差别、名字改编（mangling）、结构体内存布局、ABI 是什么、静态初始化顺序陷阱。最后接回 CMake/ament 与 ROS2、P4/P6 的交叉编译。八股速查见 [Linux、Git、编译构建八股](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)、[C 语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)、[C++ 八股](../../03_八股知识/01_计算机基础/05_C++八股.md)。

**本章学完你能：**

- 说出预处理、编译、汇编、链接各阶段的输入输出，并用 `gcc -E / -S / -c` 逐步复现；
- 读 `nm` 的符号类型（`T`、`D`、`B`、`U`、`C`），解释 `undefined reference` 与 `multiple definition` 的成因；
- 手算一次重定位：把目标文件里的 `call add` 补成正确的 32 位偏移；
- 比较静态库与动态库（链接时机、加载、更新、体积），解释 PLT/GOT 的延迟绑定思想与库搜索路径；
- 解释 C++ 名字改编、`extern "C"`、ODR，以及头文件里定义函数的后果；
- 计算结构体对齐与大小，解释为什么结构体直接发送到网络/CAN 有风险；
- 解释 ABI 与 API 的区别，说明 `libstdc++` 的 ABI 变化、交叉编译库不匹配的症状；
- 用英文回答 "What happens when you compile and run a C program?" "Static vs dynamic linking?"

**前置知识**：[操作系统](./02_操作系统_进程线程_内存_调度_同步.md)（进程地址空间布局）；C 语言基础。

**相关项目**：[CMake、ament 构建系统与依赖管理](../12_机器人软件/11_CMake_ament_构建系统与依赖管理.md)、[Docker 与部署、交叉编译](../12_机器人软件/12_Docker与部署_交叉编译_systemd.md)、[Cortex-M 体系：启动、链接脚本、内存映射](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)、[P6 OpenHarmony 基础与构建](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md)。

## 目录

1. 要解决什么问题
2. 符号与术语
3. 四个阶段：从 `.c` 到可执行文件
4. 符号、目标文件与链接
5. 重定位：手算一次 `call`
6. 静态库、动态库与加载
7. C++ 的名字改编与 ODR
8. 结构体布局、对齐与 ABI
9. 完整实验记录（本机运行）
10. 在七个项目里怎么用
11. 常见错误
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

写 C/C++ 的人最先遇到、也最容易被卡住的是：

- 编译通过，链接报 `undefined reference to 'add'`——谁没定义？在哪里该定义？
- 链接报 `multiple definition of 'x'`——为什么头文件里的一个变量会"重复"？
- 程序能编译，运行时 `error while loading shared libraries: libfoo.so`——加载器去哪里找库？
- 升级了某个系统库，没改自己代码，程序却崩溃——ABI 变了。
- 把结构体直接 `memcpy` 进 CAN 帧，换了一个编译器收到的数据错位——对齐与填充。

这些都是**翻译流水线**与**二进制接口**的问题。理解它们，调试时能从"盲目改 CMakeLists"变成"看一眼 `nm` 就知道少了哪个符号"。

## 2. 符号与术语

| 术语 | 含义 |
|---|---|
| 翻译单元（translation unit） | 一个 `.c/.cpp` 文件经预处理（展开 `#include`、宏）后的内容；编译器一次处理一个翻译单元 |
| 目标文件（object file，`.o/.obj`） | 编译器输出：机器码 + 符号表 + 重定位信息，**尚未确定最终地址** |
| 符号（symbol） | 函数或全局变量的名字，链接器用名字把引用和定义对上 |
| 节 / 段（section / segment） | 目标文件内部按用途分的区域：`.text`（代码）、`.data`（已初始化全局）、`.bss`（未初始化全局，不占文件空间）、`.rodata`（只读常量） |
| 重定位（relocation） | 链接器在确定地址后，回填目标文件中占位的地址字段 |
| 强/弱符号 | 函数和已初始化全局变量是强符号；未初始化全局可作弱符号（"common"） |
| 静态库 `.a` | 目标文件的归档，链接时把用到的成员**复制进**可执行文件 |
| 动态库 `.so`（Linux）/ `.dll`（Windows） | 运行时由加载器映射、可被多个进程共享 |
| ABI | 应用二进制接口：调用约定、数据布局、符号命名、可执行文件格式等**二进制层面**的约定 |
| API | 源代码层面的接口（函数签名、类型） |
| ODR | 单一定义规则（One Definition Rule）：同一个实体在整个程序里只能有一个定义 |
| ELF / PE / Mach-O | Linux / Windows / macOS 的可执行文件格式 |

> **说明**：本章的实验在 Windows 上的 MinGW-w64 GCC 8.1.0 中运行，产物是 PE/COFF 格式而不是 Linux 的 ELF。**阶段划分、符号概念、重定位原理、名字改编（GCC 采用 Itanium C++ ABI）、结构体对齐规则在两者间一致**；涉及 `.so`、`LD_LIBRARY_PATH`、`ldd`、`readelf` 的内容是 Linux/ELF 的通用知识，本机没有跑，标注"未在本机复核"。

## 3. 四个阶段：从 `.c` 到可执行文件

```text
 main.c ──预处理(cpp)──> main.i ──编译(cc1)──> main.s ──汇编(as)──> main.o ──┐
                                                                              ├─链接(ld)─> app
 util.c ──预处理──────> util.i ──编译──────> util.s ──汇编──────> util.o ──┘
                                                       (+ 库: libc 等)
```

| 阶段 | 命令 | 做什么 | 常见错误 |
|---|---|---|---|
| 预处理 | `gcc -E` | 展开 `#include`、`#define`、条件编译，删注释；纯文本替换 | 找不到头文件、宏副作用 |
| 编译 | `gcc -S` | 语法/语义检查，生成汇编，做优化 | 语法错误、类型错误、未声明的函数 |
| 汇编 | `gcc -c` | 汇编转机器码，生成目标文件 | 很少 |
| 链接 | `gcc a.o b.o -o app` | 合并目标文件，解析符号，重定位，生成可执行文件 | `undefined reference`、`multiple definition` |

### 3.1 预处理做了什么

对本章的 `main.c`（含 `#include "util.h"`）执行 `gcc -E main.c`，输出末尾可见 `util.h` 被展开成 `extern int g_count;` 等声明，之后是 `main.c` 自己的代码。`#include` 就是**文本复制**，所以头文件应该有**包含保护**（`#ifndef UTIL_H ... #endif` 或 `#pragma once`），否则多次包含导致重定义。

宏的陷阱：`#define SQR(x) x*x` 则 `SQR(a+1)` 展开为 `a+1*a+1`，必须写 `((x)*(x))`；宏参数多次求值，`SQR(i++)` 有副作用。C++ 里优先用 `constexpr`、`inline` 函数代替宏。

### 3.2 编译做了什么

编译一个翻译单元时，编译器**只知道声明**。看到 `add(1,2)` 而头文件里有 `int add(int,int);`，就能做类型检查并生成一条"调用 `add`"的指令，但**不知道 `add` 在哪**，它把地址字段留空，并在目标文件里记录一条"这里需要填 `add` 的地址"（重定位项）。这就是**声明与定义分离**的意义：编译只需要声明，链接才需要定义。

## 4. 符号、目标文件与链接

### 4.1 看符号表：`nm`

本章实验的 `util.c` 与 `main.c`（节选）：

```c
/* util.c */
int g_count = 7;                 /* 已初始化全局，导出 */
static int hidden = 3;           /* static：内部链接，不导出 */
int add(int a, int b) { g_count++; return a + b + hidden; }

/* main.c */
int bss_arr[100];                /* 未初始化全局 */
const char *msg = "hello";
int main(void) { printf("%d %d %s\n", add(1,2), g_count, msg); return 0; }
```

`gcc -c` 后用 `nm` 查看（本机输出，省略与实验无关的节符号）：

```text
$ nm util.o
0000000000000000 T add
0000000000000000 D g_count
0000000000000004 d hidden

$ nm main.o
                 U add
0000000000000190 C bss_arr
                 U g_count
0000000000000000 T main
0000000000000000 D msg
                 U printf
```

符号类型字母：大写表示**全局（外部可见）**，小写表示**局部**。

| 字母 | 含义 |
|---|---|
| `T` / `t` | 代码段（`.text`）里的定义 |
| `D` / `d` | 已初始化数据段（`.data`）里的定义 |
| `B` / `b` | 未初始化数据（`.bss`）里的定义 |
| `R` / `r` | 只读数据 |
| `U` | **未定义**：本文件引用了，但定义在别处，等链接器去找 |
| `C` | common 符号：未初始化的全局变量，多个文件可以同名，链接时合并 |

观察：

1. `util.o` 里 `add`、`g_count` 是 `T`/`D`（导出），`hidden` 是小写 `d`（`static`，不导出，别的文件看不到，这就是 C 里的"内部链接"）；
2. `main.o` 里 `add`、`g_count`、`printf` 是 `U`——只有引用，没有定义；
3. `bss_arr` 显示为 `C`：本机 GCC 8.1 默认把未初始化的全局变量当 common 处理，所以**两个文件各写一个 `int x;` 链接不会报错**；GCC 10 起默认改为 `-fno-common`，会报 `multiple definition`（版本差异，请以自己的编译器为准，此处版本变化时间点未逐版核对）。这是"老代码在新编译器上突然链接失败"的常见原因。

链接器的任务：把所有 `U` 在别的目标文件或库的 `T/D/B` 中找到定义，**每个符号恰好一个定义**。

### 4.2 两类链接错误

**`undefined reference`**：有 `U` 找不到定义。本机实验：只链接 `main.o`、不链接 `util.o`：

```text
$ gcc m2.o -o fail.exe
m2.o:main.c:(.text+0x2a): undefined reference to `add'
m2.o:main.c:(.rdata$.refptr.g_count[.refptr.g_count]+0x0): undefined reference to `g_count'
collect2.exe: error: ld returned 1 exit status
```

常见原因：忘了把 `.o` 或库加进命令；**库的顺序错**（见 6.1）；声明了没定义；C++ 里用 `extern "C"` 不一致（7.2）；函数定义在 `#ifdef` 之外被排除。注意这是**链接器**报的，不是编译器——所以改头文件通常无用，要找定义。

**`multiple definition`**：同一符号有两个强定义。典型：在**头文件里定义**了非 `inline`/`static` 的函数或变量，被两个 `.c` 包含，每个翻译单元都生成一份强符号。解决：头文件只放声明（`extern int x;`），定义放在一个 `.c`；或函数声明为 `static inline`（C）/ `inline`（C++）。

## 5. 重定位：手算一次 `call`

`main.o` 中调用 `add(1,2)` 的反汇编（`objdump -dr main.o`，节选）：

```text
  29:  e8 00 00 00 00     callq  2e <main+0x2e>
         2a: R_X86_64_PC32   add
```

`e8` 是 x86-64 的"近相对调用"，后面 4 字节是**相对于下一条指令地址的有符号偏移**，现在是 `00 00 00 00` 占位；下面那行是重定位项：在偏移 `0x2a` 处按 `PC32`（32 位 PC 相对）规则回填符号 `add`。

PC32 的计算公式：

$$\text{值}=S+A-P$$

- $S$：符号的最终地址；
- $A$：加数（addend），对 x86-64 的 PC 相对调用通常为 $-4$（因为偏移是相对**下一条指令**，而重定位点距离下一条指令还有 4 字节）；
- $P$：被修补位置的最终地址。

链接后（`objdump -d app3.exe`）：`main` 位于 `0x401580`，`add` 位于 `0x401550`。则：

- $P=0x401580+0x2a=0x4015aa$；
- $S=0x401550$，$A=-4$；
- 值 $=0x401550-4-0x4015aa=-0x5e$，补码 32 位为 `0xFFFFFFA2`，小端字节序写成 `a2 ff ff ff`。

实测链接后的指令是 `e8 a2 ff ff ff   callq 401550 <add>`，与手算一致。验证另一种理解：CPU 执行 `call` 时取下一条指令地址 $0x4015ae$，加上偏移 $-0x5e$，得 $0x401550$，即 `add`。

这个例子说明：**链接器的本质工作是 (1) 把各目标文件的节拼接并分配最终地址，(2) 按重定位项回填地址**。静态链接完成后，地址就固定了（或在加载时再整体平移，见位置无关代码）。

## 6. 静态库、动态库与加载

### 6.1 静态库 `.a`

用 `ar rcs libutil.a util.o` 把目标文件打包，再 `gcc main.o -L. -lutil -o app2`，本机运行 `6 7 hello`，与直接链接目标文件结果相同。链接时，链接器只拉取**被需要的成员**（`util.o` 里至少有一个符号被引用才会被拉入）。

**库的顺序问题**：链接器从左向右扫描，遇到静态库时，只提取当前**已经出现的未定义符号**需要的成员。如果写成 `gcc -lutil main.o`，扫描 `libutil.a` 时还没有任何未定义符号，什么都不取；之后 `main.o` 的 `add` 无处可找，报 `undefined reference`。规则：**依赖者在前，被依赖的库在后**（`main.o` 在 `-lutil` 之前；若库之间互相依赖需要排序或重复）。

### 6.2 动态库

动态库不把代码复制进可执行文件，只记录"需要 `libfoo.so`、需要符号 `add`"。运行时由**动态加载器**（Linux 上是 `ld.so`）完成：

1. 内核加载可执行文件时发现它声明了解释器（`PT_INTERP`），先启动动态加载器；
2. 加载器按搜索顺序找到依赖库：可执行文件内嵌的 RPATH/RUNPATH → 环境变量 `LD_LIBRARY_PATH` → 系统缓存 `/etc/ld.so.cache`（`ldconfig` 生成）→ 默认目录（`/lib`、`/usr/lib` 等）；
3. 把库**映射**进进程地址空间（`mmap`），多个进程的代码段共享同一份物理页；
4. 解析符号并重定位。

调试工具：`ldd app` 列出依赖及找到的路径（缺失会显示 `not found`）；`readelf -d app` 看 `NEEDED`、`RPATH`。以上为 Linux 通用知识，未在本机（Windows）复核。

### 6.3 位置无关代码与 PLT/GOT

共享库被加载到的地址不固定，且要被多个进程以不同地址映射，所以代码编译成**位置无关代码（PIC，`-fPIC`）**：用 PC 相对寻址访问自己的数据，对外部符号则通过**全局偏移表（GOT）**间接访问。函数调用通过**过程链接表（PLT）**：第一次调用时跳到加载器解析真实地址并写入 GOT，之后的调用直接跳转（**延迟绑定**）。这使得启动更快、库可共享，代价是每次调用多一次间接跳转。

### 6.4 静态 vs 动态：怎么选

| 维度 | 静态链接 | 动态链接 |
|---|---|---|
| 可执行文件大小 | 大 | 小 |
| 部署 | 单文件，几乎无依赖 | 需保证目标机有正确版本的库 |
| 更新库（安全补丁） | 需重新编译链接 | 替换 `.so` 即可（若 ABI 兼容） |
| 内存 | 每个进程一份 | 多进程共享代码页 |
| 启动 | 较快 | 需加载和解析 |
| 版本冲突 | 无 | "DLL hell"、符号冲突 |

嵌入式裸机（STM32）几乎只有静态链接，链接脚本决定各节放到 Flash/RAM 的哪里（见 [Cortex-M 体系](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)）；ROS2 在 Linux 上大量使用动态库（每个包生成 `.so`，节点运行时按 `LD_LIBRARY_PATH` 与 ament 前缀路径加载）。

### 6.5 可执行文件的各节大小

`size util.o main.o` 的本机输出：

```text
   text    data     bss     dec     hex filename
    132      16       0     148      94 util.o
    204      40       0     244      f4 main.o
```

`text` 为代码与只读常量，`data` 为已初始化数据，`bss` 为未初始化数据（不占文件空间，加载时由系统清零）。此处 `bss_arr` 是 common 符号，链接前不计入 `bss`，链接后才分配。在 MCU 上，这三个数字（再加栈、堆）直接对应 Flash 与 RAM 占用：Flash ≈ `text`+`data`（初值要存在 Flash 里，启动代码拷贝到 RAM），RAM ≈ `data`+`bss`+栈+堆。这也是启动文件中"把 `.data` 从 Flash 拷到 RAM、把 `.bss` 清零"这两段代码的由来。

## 7. C++ 的名字改编与 ODR

### 7.1 名字改编（name mangling）

C 没有函数重载，符号名就是函数名。C++ 允许重载、命名空间、类成员函数，所以编译器把参数类型等编码进符号名。本机对 `cpp.cpp` 编译后用 `nm`：

```text
$ nm cpp.o | grep " T "
..._Z3addii        <- int add(int,int)
..._Z3adddd        <- double add(double,double)
..._ZN2ns1fEi      <- ns::f(int)
..._ZN1S1mEi       <- S::m(int)
..._Z5S_usev       <- S_use()
..._c_add          <- extern "C" 的 c_add：不改编
$ nm -C cpp.o       （-C 解码）
T add(double, double)
T add(int, int)
T S::m(int)
T ns::f(int)
```

读法：`_Z` 前缀，`3add` 表示长度 3 的名字 `add`，`ii` 是两个 `int` 参数，`dd` 是两个 `double`；`_ZN2ns1fEi` 中 `N...E` 包裹嵌套名 `ns::f`，参数 `i`。**返回类型通常不编进普通函数的符号名**，所以两个仅返回类型不同的重载不合法。（编码规则属于 Itanium C++ ABI，GCC/Clang 在 Linux 上通用，MSVC 使用不同的规则。）

### 7.2 `extern "C"`

C++ 要调用 C 库（或让 C 调用 C++ 函数），需要关闭改编：

```cpp
#ifdef __cplusplus
extern "C" {
#endif
int c_add(int a, int b);     /* 符号名就是 c_add */
#ifdef __cplusplus
}
#endif
```

C++ 代码若声明了 C 库函数却忘了 `extern "C"`，链接时会找 `_Z5c_addii`，而 C 库里只有 `c_add`，报 `undefined reference`。这也是跨语言 ABI（Python 的 `ctypes`、ROS 中 C 接口的驱动库）都使用 C 链接的原因。

### 7.3 ODR 与头文件中的定义

- 类、模板、内联函数可以在多个翻译单元中定义，但必须**逐字相同**；违反时多数情况链接器不会报错，行为是未定义（"静默地用了其中一份"），极难排查。
- 非内联函数、全局变量只能有一个定义。C++17 起 `inline` 变量可以在头文件里定义。
- 头文件里的 `static` 全局变量不会冲突，但每个翻译单元一份拷贝，修改互不可见——常见的"我改了全局变量别的文件没变"的原因。
- 模板定义必须在使用处可见（放在头文件），因为实例化发生在编译期的使用点。

### 7.4 静态初始化顺序

不同翻译单元中全局对象的构造顺序**未规定**。若 `a.cpp` 的全局对象构造时用到 `b.cpp` 的全局对象，可能用到未构造的对象（static initialization order fiasco）。对策：把全局对象放进函数的静态局部变量（首次调用时构造，C++11 起线程安全），即"Meyers 单例"形态。

## 8. 结构体布局、对齐与 ABI

### 8.1 对齐规则

每个基本类型有对齐要求（通常等于其大小：`char` 1、`short` 2、`int` 4、`double` 8，x86-64 常见）；结构体成员按声明顺序排列，每个成员的偏移必须是其对齐的倍数，不足处插入**填充（padding）**；结构体总大小是最大成员对齐的倍数。

本机实验（x86-64，GCC）：

```c
struct A { char a; int b; char c; };   /* 偏移: a=0, b=4, c=8, 总大小 12 */
struct B { int b; char a; char c; };   /* 偏移: b=0, a=4, c=5, 总大小 8 */
#pragma pack(push,1)
struct P { char a; int b; char c; };   /* 取消填充，总大小 6 */
#pragma pack(pop)
```

`sizeof` 输出 `A 12 (b@4 c@8) B 8 P 6`，与手算一致。

**手算 A**：`a` 在 0，占 1 字节；`b` 需 4 字节对齐，跳过 3 字节填充，偏移 4，占到 8；`c` 在 8，占到 9；总大小要补成 4 的倍数：12。**把成员按大小从大到小排**（`B`）可省空间。

### 8.2 为什么不要直接发送结构体

1. 不同编译器/平台**填充和对齐不同**；
2. **字节序**不同（STM32 小端、网络序大端）；
3. 编译器版本或编译选项改动成员布局；
4. 指针成员发送过去毫无意义。

做法：显式序列化（逐字段按约定字节序写入缓冲）；或使用 `#pragma pack(1)`/`__attribute__((packed))` 加上 `static_assert(sizeof(T)==N)`，同时意识到 packed 结构的成员访问在某些架构上会产生非对齐访问（Cortex-M0 不支持，M3/M4/M7 部分支持但有性能代价，具体以芯片手册为准）。CAN 帧的 8 字节数据区、电机私有协议的字段打包都是这个问题（见 [CAN 与电机通信](../11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)）。

### 8.3 ABI 与 API

**API 兼容**：源代码不改即可重新编译。**ABI 兼容**：不重新编译，旧二进制就能和新库一起工作。ABI 包括：

- **调用约定**：参数通过哪些寄存器/栈传递、返回值放哪、谁清理栈、哪些寄存器由被调用者保存（System V AMD64 与 Windows x64 不同，AArch64 和 ARM Cortex-M 的 AAPCS 又不同）；
- **类型布局**：成员偏移、虚函数表布局、`std::string` 等标准库类型的内部表示；
- **符号命名**与名字改编规则；
- **异常处理**机制、RTTI 布局。

典型问题：**GCC 5 引入新的 `std::string`/`std::list` 实现（C++11 ABI），通过宏 `_GLIBCXX_USE_CXX11_ABI` 控制**；用新旧不同 ABI 编译的库混合链接，会出现 `undefined reference to ...__cxx11::basic_string...` 类的错误。**交叉编译**时，目标板上的 `libstdc++`、`glibc` 版本必须不低于交叉工具链链接时使用的版本，否则运行时报 `GLIBC_2.xx not found` 或 `GLIBCXX_3.4.xx not found`（P4 的 RK3588、P6 的 OpenHarmony 设备上部署自编译程序时的常见问题类型；具体版本号因工具链而异，未核对）。

**保持库的 ABI 稳定**的做法：在公共头文件里只暴露 C 接口，或使用 PIMPL 惯用法（公共类只持有一个指向实现的指针，实现细节放 `.cpp`），这样加减私有成员不改变公共类的大小与布局。

### 8.4 符号可见性

动态库默认导出所有非 `static` 符号，容易与别的库同名符号冲突、也拖慢加载。用 `-fvisibility=hidden` 配合显式 `__attribute__((visibility("default")))` 只导出公共接口。ROS2 包里常见的 `XXX_PUBLIC` 宏（形如 `visibility_control.h`）就是这个用途（由 `ament_cmake` 的模板生成，Windows 下对应 `__declspec(dllexport)`）。

## 9. 完整实验记录（本机运行）

环境：Windows 11，MinGW-w64 GCC 8.1.0（posix 线程模型）。以下均为实际命令，结论已在 3–8 节引用。

```text
$ gcc -E main.c | tail -5          # 预处理：展开 #include
$ gcc -S -O0 util.c -o util.s      # 编译到汇编：add 里先 g_count++ 再相加，读 hidden
$ gcc -c util.c main.c             # 汇编到目标文件
$ nm util.o ; nm main.o ; size util.o main.o
$ gcc util.o main.o -o app.exe && ./app.exe        # 输出: 6 7 hello
$ gcc m2.o -o fail.exe             # 少一个 .o：undefined reference to `add'
$ ar rcs libutil.a util.o && gcc main.o -L. -lutil -o app2.exe && ./app2.exe   # 输出: 6 7 hello
$ g++ -c cpp.cpp ; nm cpp.o | grep " T " ; nm -C cpp.o | grep " T "
$ gcc lay.c -o lay.exe && ./lay.exe                # 输出: A 12 (b@4 c@8) B 8 P 6
$ objdump -dr main.o ; objdump -d app3.exe         # 重定位前后对比
```

验证输出 `6 7 hello`：`add(1,2)=1+2+hidden(3)=6`；`g_count` 初值 7，但 printf 的参数求值顺序未规定——本次编译器先取了 `g_count`（7）再调用 `add`，因此打印 7 而不是 8。这恰好是一个小而真实的"未规定行为"例子：**函数调用参数的求值顺序在 C/C++ 中是未规定的**，不要在同一个表达式里既修改又读取同一个变量。

## 10. 在七个项目里怎么用

| 项目 | 对应概念 | 性质 |
|---|---|---|
| P3/P7（STM32） | 裸机/RTOS 固件是静态链接：启动文件 + 链接脚本决定 `.text` 放 Flash、`.data` 初值从 Flash 拷到 RAM、`.bss` 清零；编译产物的 `text/data/bss` 大小决定 Flash 与 RAM 占用；中断向量表是链接脚本放在固定地址的一个节。见 [Cortex-M 体系](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md) | 项目内（概念） |
| P1/P2/P5（ROS2） | 每个 ROS2 包用 CMake + ament 生成 `.so` 或可执行文件，`find_package` 与 `target_link_libraries` 决定链接依赖，依赖顺序错误或缺 `ament_target_dependencies` 就是 `undefined reference`；消息类型生成的是 C++ 库，ABI 与编译器、ROS 发行版绑定（Humble 对应 Ubuntu 22.04）。见 [CMake 与 ament](../12_机器人软件/11_CMake_ament_构建系统与依赖管理.md) | 项目内 |
| P4（RK3588） | 在 PC 上交叉编译再部署到板子，或在板子上本机编译；RKNN 运行库通常以动态库（`.so`）形式提供，需要与板子系统的 glibc/libstdc++ 版本兼容；动态库搜索路径（`LD_LIBRARY_PATH`/RPATH）决定程序能否启动 | 项目内（概念，具体版本未核对） |
| P6（OpenHarmony） | 构建体系是 GN + Ninja（OpenHarmony 的构建系统，常见表述）；组件以 `.z.so`（动态库）形式组织，接口以稳定的 C/C++ 接口与 IDL 生成代码暴露，版本管理和 ABI 稳定性正是"统一能力模型"工程化的一部分。见 [P6 OpenHarmony 基础与构建](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md) | 项目内（概念） |
| P2/P3/P7 | CAN/串口协议帧的打包解包：结构体对齐、字节序，用显式序列化代替结构体直发 | 项目内（工程实践） |

> **面试要点**：被问到"遇到过什么构建/链接问题"时，可以按 `undefined reference`→看 `nm`/`ldd`→查是缺定义、缺库、库顺序、`extern "C"` 还是 ABI 版本的逻辑来讲，并强调"链接错误由链接器报告，要找定义不是声明"。

## 11. 常见错误

1. **在头文件里定义变量或非 inline 函数**：多重定义。
2. **静态库顺序写反**：依赖者放在库后面。
3. **忘记 `extern "C"`** 或两边不一致，C 与 C++ 混编时找不到符号。
4. **宏参数不加括号**、多次求值。
5. **头文件缺包含保护**。
6. **用 `-I` 路径或 `LD_LIBRARY_PATH` 临时修补而不是修好 RPATH/安装路径**，换一台机器就复现失败。
7. **混用不同 ABI 的库**（不同 `_GLIBCXX_USE_CXX11_ABI`、不同编译器）。
8. **直接把结构体发送给别的设备**。
9. **依赖全局对象的构造顺序**。
10. **在 Debug 与 Release 间混用库**（MSVC 下尤其明显；Linux 上则是 `-D_GLIBCXX_DEBUG` 等改变布局的宏）。
11. **忽略编译警告**：未声明函数的隐式声明（旧 C 标准）会让编译通过、运行出错；开 `-Wall -Wextra`。
12. **交叉编译用了主机的头文件/库**（`-I/usr/include` 污染），生成的二进制在板子上不可运行。

## 12. 英文面试问答

### Q1. What happens when you compile and run a C program?

"There are four stages. The preprocessor expands includes and macros. The compiler turns each source file into assembly, using only declarations for functions defined elsewhere. The assembler produces object files with machine code, a symbol table and relocation entries. The linker merges the object files and libraries, resolves every undefined symbol to a definition, and patches the addresses. Finally, the OS loader maps the executable into a new process, the dynamic loader brings in shared libraries, and execution starts at the entry point, which calls main."

中文要点：预处理（文本展开）→ 编译（只需声明）→ 汇编（目标文件含符号表与重定位）→ 链接（解析符号、回填地址）→ 加载（映射进程、动态库、从入口开始）。

**追问**：

- *"What is the difference between a declaration and a definition?"* — "A declaration says a name exists and what its type is; a definition allocates storage or provides the body. The compiler needs declarations, the linker needs exactly one definition per symbol."
- *"What does 'undefined reference' mean and how do you debug it?"* — "The linker found a use of a symbol but no definition. I check with nm which object file should define it, whether it is in the link command and in the right order, whether it is declared extern C consistently, and whether the library was built with the same ABI."

### Q2. Static linking vs dynamic linking: when would you use each?

"Static linking copies the needed code into the executable, so deployment is simple and there are no version problems, but binaries are bigger and a security fix means relinking. Dynamic linking keeps libraries separate, saves memory by sharing code pages, and lets you update a library without rebuilding, but you must manage versions and search paths. On microcontrollers it is always static. On Linux robotics software, like ROS2 packages, it is usually dynamic."

中文要点：静态——部署简单无版本问题，但体积大、更新需重链接；动态——共享内存、可单独更新，但需管理版本与搜索路径；MCU 一律静态，ROS2 通常动态。

### Q3. What is an ABI and why can upgrading a library break a program that was not recompiled?

"The API is the source-level interface; the ABI is the binary-level contract: calling convention, struct layouts, symbol names and so on. If a library changes the layout of a class or the mangled name of a function, old binaries still assume the old layout and either fail to link or read wrong offsets at run time. For example, libstdc++ introduced a new string ABI, which is controlled by a macro. To keep a library ABI-stable you can expose a C interface or use the pimpl idiom."

中文要点：API 是源码接口，ABI 是二进制约定（调用约定、布局、符号名）；库改了布局或名字改编，旧二进制就出错；例子：libstdc++ 新 `std::string` ABI；稳定办法：C 接口或 pimpl。

### Q4. Why shouldn't you send a C struct directly over CAN or a network?

"Because the in-memory layout isn't a protocol. Different compilers insert different padding, different CPUs have different byte order, and pointers mean nothing on the other side. I serialize field by field with a defined byte order, or I use a packed struct with a static assert on its size, and I write the field layout in the protocol document."

中文要点：内存布局不是协议；填充、字节序、指针都有问题；逐字段按约定序列化，或 packed + `static_assert`，并在协议文档中写明布局。

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 四阶段：预处理（文本）→ 编译（要声明）→ 汇编（目标文件）→ 链接（要定义）。
2. 目标文件 = 机器码 + 符号表 + 重定位信息；`nm` 的 `T/D/B` 是定义，`U` 是待解析引用。
3. `undefined reference` 是链接器报：找不到定义；`multiple definition`：头文件里定义了东西。
4. 重定位公式 $S+A-P$；x86-64 `call` 的加数为 $-4$；本章手算 `0xFFFFFFA2` 与实测一致。
5. 静态库顺序：依赖者在前；静态库只拉取被需要的成员。
6. 动态库：加载器按 RPATH、`LD_LIBRARY_PATH`、缓存、默认目录找；PLT/GOT 做延迟绑定。
7. C++ 名字改编把参数类型编进符号；`extern "C"` 关闭改编，C/C++ 互调必备。
8. ODR：同一实体全程序一个定义；全局对象的跨文件构造顺序未规定。
9. 结构体对齐：成员偏移是自身对齐的倍数，总大小是最大对齐的倍数；`{char,int,char}` 为 12，调序后为 8。
10. ABI 是二进制层面的约定；改布局或改名字改编会破坏 ABI；不要直接发送结构体。

### 自测题

1. 解释为什么 `gcc -lutil main.o` 链接失败而 `gcc main.o -lutil` 成功。
   *要点*：链接器从左到右处理，只为已出现的未定义符号从静态库提取成员。
2. 计算 `struct { double d; char c; int i; }` 在 x86-64 的大小与各偏移。
   *要点*：d@0（8）；c@8（1）；i 对齐 4，偏移 12；总 16（最大对齐 8，16 是 8 的倍数）。
3. 某 `call` 指令的重定位点地址 $P=0x1004$，目标函数 $S=0x2000$，加数 $A=-4$，求填入的 32 位值。
   *要点*：$0x2000-4-0x1004=0xFF8$，即 `f8 0f 00 00`（小端）。
4. 为什么两个 `.cpp` 文件各自在头文件里写 `int counter = 0;` 会链接失败，而写 `static int counter = 0;` 不会？它们的语义有何不同？
   *要点*：前者各生成一个强符号 `counter`，重复定义；后者各自内部链接、各有一份，互不可见，改一个不影响另一个。
5. 为什么 ROS2 的 C++ 包升级编译器后可能出现无法运行的旧二进制？
   *要点*：ABI（标准库实现、名字改编、布局）变化；要用同一发行版、同一编译器构建整个依赖链。

### 相关链接

- 八股速查：[Linux、Git、编译构建八股](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)、[C 语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)
- 上一章：[并发与内存模型](./05_并发与内存模型_锁_原子_无锁.md)；下一章：[数据库与分布式基础](./07_数据库与分布式基础_一致性_时钟.md)
- 相关：[CMake、ament](../12_机器人软件/11_CMake_ament_构建系统与依赖管理.md)、[C++ 现代特性与工程实践](../12_机器人软件/02_C++现代特性与工程实践.md)、[Docker 与部署、交叉编译](../12_机器人软件/12_Docker与部署_交叉编译_systemd.md)
- [模块 README](./README.md)
