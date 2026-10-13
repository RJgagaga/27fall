# Linux、Git 与编译构建八股

> **定位**：机器人开发的日常工具链（CV 技能栏：Linux、Git、C/C++、ROS/ROS2）。面试常问：Linux 常用命令、进程/信号/管道、文件权限、Makefile/CMake、静态与动态链接、Git 常见操作。本章答案都尽量给出"命令 + 原理 + 排障思路"。

**本章学完你能……**
- 用命令定位 CPU、内存、端口、日志问题；
- 解释进程、信号、管道、重定向、权限位；
- 说清源码到可执行文件的四个阶段，静态库/动态库区别，CMake 的基本写法；
- 熟练回答 Git 的 merge/rebase/cherry-pick/reset/revert 等问题。

**前置知识**：[操作系统](./02_操作系统.md)。基础补充：[Linux与Shell_Git](../../02_基础知识/09_机器人软件/01_Linux与Shell_Git.md)。

---

## 一、Linux 常用命令与排障

### Q1 Linux 常用命令有哪些？按场景分类说一下。（Common commands）
**优先级：★★★**

| 场景 | 命令 |
|---|---|
| 文件与目录 | `ls -lah`、`cd`、`pwd`、`cp -r`、`mv`、`rm -rf`（慎用）、`mkdir -p`、`ln -s` |
| 查看内容 | `cat`、`less`、`head`、`tail -f`（跟随日志）、`wc -l` |
| 查找 | `find . -name "*.cpp"`、`grep -rn "pattern" .`、`locate`、`which` |
| 文本处理 | `sed`、`awk`、`sort`、`uniq -c`、`cut`、`xargs`（管道组合使用） |
| 进程 | `ps aux`、`top/htop`、`kill`、`pkill`、`nohup`、`jobs`、`bg/fg` |
| 资源 | `free -h`、`df -h`、`du -sh *`、`vmstat`、`iostat`、`lsof` |
| 网络 | `ip a`、`ping`、`ss -tlnp`、`curl`、`tcpdump`、`netstat` |
| 权限 | `chmod`、`chown`、`sudo` |
| 压缩 | `tar -czf`、`tar -xzf`、`zip/unzip` |
| 设备 | `lsusb`、`lsblk`、`dmesg`、`ls /dev/tty*`、`candump`（can-utils） |

**答案要点**：重点说场景而不是罗列。例：在机器人上排查"串口打不开"：`ls -l /dev/ttyUSB0`（权限）→ 用户是否在 `dialout` 组 → `lsof /dev/ttyUSB0` 看谁占用 → `dmesg | tail` 看是否枚举成功。

**English short answer**
I group commands by task: ls, find and grep for files, tail dash f for logs, ps, top and kill for processes, free and df for resources, ss and ping for networking, and chmod for permissions. For example, when a serial port will not open I check the device permissions, group membership, which process holds it with lsof, and the kernel messages with dmesg.

**追问方向**：`find` + `xargs`；`awk` 简单统计。

### Q2 怎么排查 CPU 占用高 / 内存不够 / 磁盘满？（Troubleshooting resources）
**优先级：★★★**

**答案要点**
- **CPU**：`top`（按 `P` 排序、`1` 看各核、`H` 看线程）→ 找到进程 → `top -H -p PID` 找线程 → `perf top` / `gdb -p PID` / `py-spy` 看热点函数。负载均值 `uptime` 的 load 与核数比较；`us/sy/wa` 看用户态/内核态/IO 等待。
- **内存**：`free -h` 看 `available`（不是 `free`；buff/cache 可回收）；`ps aux --sort=-%mem`；OOM 查 `dmesg | grep -i oom`；泄漏用 Valgrind/ASan/`tracemalloc`。
- **磁盘**：`df -h` 看分区，`du -sh * | sort -h` 逐层找大目录；inode 满用 `df -i`；已删除但仍被进程占用的文件用 `lsof +L1`。日志目录要做轮转（logrotate）——机器人长期运行写满存储是常见故障。
- **IO**：`iostat -x`、`iotop`。

**English short answer**
For high CPU I use top, switch to the thread view, then profile with perf or gdb to find the hot function. For memory I look at the available column in free, sort processes by memory and check dmesg for OOM kills. For disk I use df to find the full partition and du to find the large directory, and I remember that deleted files held by a process still use space.

**追问方向**：load average 的含义；swap；OOM killer 策略。

### Q3 Linux 进程相关命令与状态？（Process states in Linux）
**优先级：★★**

**答案要点**：`ps` 状态码：`R` 运行/可运行，`S` 可中断睡眠，`D` 不可中断睡眠（等 IO，`kill -9` 也杀不掉，常是磁盘/NFS/驱动问题），`T` 停止，`Z` 僵尸。`kill -l` 列信号。`nice/renice` 调优先级。`systemctl status/start/stop/enable`、`journalctl -u 服务名 -f` 管理与查看服务日志；开机自启机器人程序用 systemd unit。

```ini
# 教学示例: /etc/systemd/system/robot.service
[Unit]
Description=Robot control
After=network.target
[Service]
ExecStart=/opt/robot/bin/control
Restart=on-failure
User=robot
[Install]
WantedBy=multi-user.target
```

**English short answer**
Process states in ps include R for running, S for interruptible sleep, D for uninterruptible sleep usually waiting on IO, T for stopped and Z for zombie. I manage services with systemctl and read their logs with journalctl, and I use a systemd unit with restart on failure to run a robot program at boot.

**追问方向**：D 状态进程怎么处理；僵尸进程怎么清理。

### Q4 什么是信号？常见信号有哪些？（Signals）
**优先级：★★★**

**答案要点**：信号是内核/进程向进程发送的异步通知。常见：`SIGINT`(2，Ctrl+C)、`SIGTERM`(15，礼貌终止，可捕获，默认 `kill`)、`SIGKILL`(9，强杀，不可捕获)、`SIGSTOP`(19，暂停，不可捕获)、`SIGHUP`(1，终端断开/重载配置)、`SIGSEGV`(11，段错误)、`SIGCHLD`(子进程状态变化)、`SIGPIPE`(写已关闭管道/socket)。处理函数里只能调用异步信号安全函数（不能 `printf`/`malloc`），常做法是设置 `volatile sig_atomic_t` 标志，主循环检测后优雅退出。机器人程序收到 `SIGINT/SIGTERM` 时应先让电机失能再退出。

```c
#include <signal.h>
static volatile sig_atomic_t g_stop = 0;
static void on_sig(int s){ (void)s; g_stop = 1; }
int main(void){
    signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
    while(!g_stop){ /* 控制循环 */ }
    /* 失能执行器, 释放资源 */
    return 0;
}
```

**English short answer**
A signal is an asynchronous notification sent to a process. SIGTERM asks it to terminate and can be handled, SIGINT is Ctrl+C, SIGKILL cannot be caught, and SIGSEGV indicates an invalid memory access. In a signal handler I only set a flag, and the main loop then shuts down cleanly, for example by disabling the motors before exiting.

**追问方向**：信号处理函数为何限制多；`sigaction` 与 `signal`；ROS 的 `Ctrl+C` 处理。

### Q5 管道、重定向、`2>&1`、`tee`？（Pipes and redirection）
**优先级：★★★**

**答案要点**：`cmd1 | cmd2`：前者标准输出接后者标准输入（内核 pipe）。重定向：`>` 覆盖，`>>` 追加，`<` 输入，`2>` 标准错误。`cmd > out.log 2>&1` 把 stderr 合并到 stdout（顺序重要，先重定向 stdout 再 `2>&1`）。`tee` 同时输出到屏幕和文件。`/dev/null` 丢弃输出。`xargs` 把标准输入变成参数。`nohup cmd > log 2>&1 &` 后台运行且断开终端不退出。

```bash
ros2 launch pkg bringup.launch.py 2>&1 | tee run.log
dmesg | grep -i can | tail -n 20
```

**English short answer**
A pipe connects the standard output of one command to the standard input of the next. Redirection uses greater-than to write to a file and two greater-than to append, and two-greater-than-ampersand-one merges errors into the normal output. The order matters: I redirect stdout to the file first and then duplicate stderr onto it. tee lets me watch the output and save it at the same time.

**追问方向**：`2>&1` 顺序；管道的缓冲；命名管道 FIFO。

### Q6 文件权限 `rwx` 和 `chmod 755` 是什么意思？（File permissions）
**优先级：★★★**

**答案要点**：权限分三组：所有者 user、所属组 group、其他人 other；每组 r(4) w(2) x(1)。`755` = `rwxr-xr-x`：所有者全权限，其他人可读可执行。目录的 `x` 表示可进入。特殊位：`setuid`(4xxx，以文件所有者身份运行)、`setgid`、`sticky`(1xxx，`/tmp` 中只有所有者能删自己的文件)。`chown user:group file`，`umask` 决定新文件默认权限（`022` → 文件 644、目录 755）。串口、CAN、摄像头设备需把用户加入对应组（`dialout`、`video`）或用 udev 规则。

**English short answer**
Permissions are read, write and execute for the owner, the group and others, written as bits 4, 2 and 1. So 755 means the owner can read, write and execute and everyone else can read and execute. For directories, execute means permission to enter. For hardware devices I add the user to the right group, like dialout for serial ports, or write a udev rule.

**追问方向**：setuid 风险；ACL；udev 规则。

### Q7 `grep/sed/awk` 能做什么？举例。（Text processing）
**优先级：★★**

**答案要点**：`grep -rn "error" log/`、`grep -E`、`-v` 反选、`-A/-B` 上下文；`sed 's/old/new/g' file`、`sed -n '10,20p'`；`awk '{sum+=$3} END{print sum/NR}' file` 统计列均值；`sort | uniq -c | sort -nr` 频次统计。可用于快速分析控制日志（如提取某一列的最大值）。

```bash
awk -F, 'NR>1 {if($3>m)m=$3} END{print "max:",m}' log.csv
```

**English short answer**
grep searches text with patterns, sed does stream edits and substitutions, and awk processes columns, which is handy for quickly computing statistics like the maximum or mean of a column in a CSV log.

**追问方向**：正则；大文件处理。

### Q8 SSH、scp、rsync、tmux 在机器人开发中的使用？（Remote work tools）
**优先级：★★**

**答案要点**：`ssh user@ip` 登录机器人；密钥登录 `ssh-keygen` + `ssh-copy-id`；`scp` / `rsync -avz` 同步文件（rsync 增量、可断点）；`tmux`/`screen` 保持会话，网络断开后程序继续运行；端口转发 `ssh -L`；`~/.ssh/config` 配别名。

**English short answer**
I log into the robot with ssh using key authentication, copy files with rsync, which is incremental, and use tmux so my sessions and running programs survive a network drop. SSH port forwarding helps to reach services on the robot.

**追问方向**：SSH 密钥原理；跳板机。

### Q9 环境变量、`PATH`、`source`、`.bashrc` 的作用？（Environment variables）
**优先级：★★**

**答案要点**：环境变量是进程继承的键值对。`PATH` 决定命令搜索路径；`LD_LIBRARY_PATH` 动态库搜索路径；`export VAR=val`。`source setup.bash` 在当前 shell 中执行脚本（不是子进程），ROS 用它设置 `ROS_DISTRO`、`AMENT_PREFIX_PATH`、`PYTHONPATH`。`.bashrc` 在交互式非登录 shell 启动时读取。多个工作空间的 source 顺序会覆盖，常见 "找不到包" 问题。

**English short answer**
Environment variables are key-value pairs inherited by child processes. PATH is where commands are searched, and LD_LIBRARY_PATH is where shared libraries are searched. Sourcing a script runs it in the current shell, which is how ROS sets up its workspace, and the order of sourcing matters.

**追问方向**：`ros2 pkg list` 找不到包的排查；`env`。

### Q10 Shell 脚本基础：变量、条件、循环、退出码？（Shell scripting）
**优先级：★★**

**答案要点**：`$?` 上条命令退出码（0 成功）；`set -euo pipefail` 让脚本遇错即停；引用变量加双引号 `"$var"` 防止空格分词；`[[ -f file ]]`；`for f in *.bag; do ...; done`。

```bash
#!/usr/bin/env bash
set -euo pipefail
for f in logs/*.csv; do
    echo "$f: $(wc -l < "$f") lines"
done
```

**English short answer**
A command's exit code is zero for success, and I begin scripts with set -euo pipefail so they stop on errors and unset variables. I always quote variables to survive spaces in file names.

**追问方向**：`trap`；函数；`getopts`。

---

## 二、文件系统、IO 与调试工具

### Q11 `/proc`、`/sys`、`/dev` 是什么？（Special file systems）
**优先级：★★**

**答案要点**：`/proc`：内核导出进程和系统信息的虚拟文件系统（`/proc/PID/status`、`/proc/cpuinfo`、`/proc/meminfo`）；`/sys`：设备与驱动模型（GPIO、LED、cpufreq 可通过文件读写）；`/dev`：设备节点（`/dev/ttyUSB0`、`/dev/video0`、`/dev/null`）。嵌入式 Linux 常用 sysfs 控制 GPIO。

**English short answer**
/proc exposes process and kernel information as virtual files, /sys exposes the device and driver model, and /dev holds device nodes such as serial ports and cameras. On embedded Linux I can control GPIO or read temperatures through sysfs files.

**追问方向**：`/proc/PID/fd`；设备树。

### Q12 如何调试一个段错误（Segmentation fault）？（Debugging segfaults）
**优先级：★★★**

**答案要点**：①编译加 `-g`（并 `-O0` 或 `-Og`）；②`ulimit -c unlimited` 打开 core dump，`gdb ./prog core` → `bt` 看调用栈；③`gdb ./prog` 直接运行 `r`，崩溃后 `bt`、`frame N`、`print var`；④AddressSanitizer `-fsanitize=address` 定位越界/UAF；⑤Valgrind。常见原因：空指针、野指针、数组越界、栈溢出（无限递归）、释放后使用、释放后写。

```bash
g++ -g -O0 -fsanitize=address main.cpp -o main
gdb --args ./main arg1
(gdb) run
(gdb) bt
```

**English short answer**
I build with debug symbols, and run the program under gdb or open the core dump, then use bt to see the call stack at the crash. AddressSanitizer is great for finding out-of-bounds access or use-after-free. Common causes are null or dangling pointers, array overruns and infinite recursion.

**追问方向**：gdb 常用命令；断点条件；`catch throw`。

### Q13 gdb 常用命令？（gdb basics）
**优先级：★★**

**答案要点**：`break file:line`/`b func`、`run`、`next`(n)、`step`(s)、`continue`(c)、`finish`、`print x`/`p *ptr`、`display`、`watch var`（数据断点）、`bt`、`info locals`、`thread apply all bt`、`set var x=1`、`x/16xb addr`（看内存）。嵌入式用 `arm-none-eabi-gdb` + OpenOCD/ST-Link。

**English short answer**
The essentials are break, run, next, step, continue, print, watch for data breakpoints, bt for the call stack, and thread apply all bt for multi-threaded programs. For microcontrollers I use arm-none-eabi-gdb with OpenOCD.

**追问方向**：远程调试 gdbserver。

### Q14 `strace`、`ltrace`、`perf`、`valgrind` 各用来干什么？（Tracing and profiling tools）
**优先级：★★**

**答案要点**：`strace`：跟踪系统调用（程序卡在哪个 `read`/`futex`、打不开哪个文件）；`ltrace`：库调用；`perf top/record/report`：采样热点函数，火焰图；`valgrind --tool=memcheck`：内存错误和泄漏（慢 10–50 倍），`callgrind`、`helgrind`（线程）；`ldd` 查依赖库，`nm`/`objdump`/`readelf` 看符号和节。

**English short answer**
strace shows the system calls a process makes, so I can see where it hangs or which file it cannot open. perf samples the CPU to find hot functions, and valgrind finds memory errors and leaks. ldd shows which shared libraries an executable needs.

**追问方向**：火焰图；`LD_DEBUG`。

---

## 三、编译、链接与构建

### Q15 从源码到可执行文件经过哪些阶段？（Compilation stages）
**优先级：★★★**

```text
main.c --预处理(cpp)--> main.i --编译(cc1)--> main.s --汇编(as)--> main.o --链接(ld)--> a.out
```

**答案要点**
1. **预处理**：展开 `#include`、`#define`、条件编译，删注释（`gcc -E`）。
2. **编译**：词法/语法/语义分析、优化，生成汇编（`gcc -S`）。
3. **汇编**：汇编转机器码，生成可重定位目标文件 `.o`（`gcc -c`）。
4. **链接**：合并多个 `.o` 和库，符号解析与重定位，得到可执行文件。
- 错误对应：语法错 → 编译期；`undefined reference` → 链接期；`cannot open shared object file` → **运行期**动态加载。
- 交叉编译：`arm-none-eabi-gcc`（裸机）、`aarch64-linux-gnu-g++`（如 RK3588 上的 Linux）。

**English short answer**
There are four stages: preprocessing expands includes and macros, compilation turns code into assembly, assembly produces object files, and linking resolves symbols and combines the object files and libraries into the executable. Undefined reference errors come from the linker, while a missing shared library error happens at load time.

**追问方向**：头文件和源文件分工；符号可见性；交叉编译工具链。

### Q16 静态库与动态库的区别？（Static vs shared libraries）
**优先级：★★★**

| 项 | 静态库 `.a` | 动态库 `.so` |
|---|---|---|
| 链接时机 | 链接期拷贝进可执行文件 | 运行期加载（或启动时） |
| 体积 | 可执行文件大 | 小，多进程共享一份代码 |
| 更新 | 需重新链接 | 替换 `.so` 即可（ABI 兼容时） |
| 部署 | 独立，无依赖问题 | 需保证库路径与版本 |
| 位置无关 | 不一定 | 需 `-fPIC` |
| 典型问题 | 符号重复、体积 | `LD_LIBRARY_PATH`、`undefined symbol`、版本冲突 |

**答案要点**：创建：`ar rcs libfoo.a a.o b.o`；`gcc -shared -fPIC -o libfoo.so a.c b.c`。链接：`-L路径 -lfoo`（`-l` 会找 `libfoo.so`/`libfoo.a`）。动态库运行时搜索顺序：`RPATH/RUNPATH` → `LD_LIBRARY_PATH` → `/etc/ld.so.cache`（`ldconfig`）→ 默认目录。静态库链接顺序：依赖者在前，被依赖者在后。嵌入式裸机通常静态链接；ROS 的包以动态库为主。

**English short answer**
A static library is copied into the executable at link time, so the program is self-contained but larger. A shared library is loaded at run time and shared between processes, so updating is easy, but I have to make sure the right version is found, via rpath, LD_LIBRARY_PATH or ldconfig. Shared libraries must be built as position independent code.

**追问方向**：延迟绑定 PLT/GOT；符号冲突；ABI 兼容。

### Q17 动态链接是怎么工作的？PLT/GOT 是什么？（Dynamic linking）
**优先级：★★**

**答案要点**：可执行文件中对共享库函数的调用经 PLT（过程链接表）跳转，目标地址存在 GOT（全局偏移表）。首次调用通过动态链接器解析符号并填入 GOT（延迟绑定 lazy binding），之后直接跳转。好处：库代码可被多进程共享、加载地址随机（ASLR）。`LD_PRELOAD` 可优先加载自己的库以替换函数。

**English short answer**
Calls into a shared library go through the procedure linkage table, which jumps to an address stored in the global offset table. On the first call the dynamic linker resolves the real address and fills it in, which is lazy binding. This allows code to be shared and loaded at random addresses.

**追问方向**：`LD_PRELOAD`；符号版本。

### Q18 Makefile 基本语法与原理？（Makefile）
**优先级：★★**

**答案要点**：规则 `target: prerequisites` + Tab 缩进的命令。make 比较时间戳，只重建过期目标（增量编译）。常用变量：`$@`（目标）、`$<`（第一个依赖）、`$^`（所有依赖）；模式规则 `%.o: %.c`；`.PHONY` 声明伪目标；`-MMD -MP` 自动生成头文件依赖，否则改头文件不会触发重编译。

```make
CC := gcc
CFLAGS := -Wall -O2 -MMD -MP
SRC := $(wildcard src/*.c)
OBJ := $(SRC:.c=.o)
app: $(OBJ)
	$(CC) $^ -o $@
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@
-include $(OBJ:.o=.d)
.PHONY: clean
clean:
	rm -f app $(OBJ) $(OBJ:.o=.d)
```

**English short answer**
A Makefile consists of rules with a target, its prerequisites and the commands to build it. make compares timestamps and rebuilds only what is out of date. Automatic variables like dollar-at and dollar-less-than refer to the target and the first prerequisite, and I generate dependency files so header changes trigger rebuilds.

**追问方向**：为什么改头文件没重编译；并行 `make -j`。

### Q19 CMake 是什么？基本写法？（CMake basics）
**优先级：★★★**

**答案要点**：CMake 是跨平台的**构建系统生成器**：读 `CMakeLists.txt`，生成 Makefile/Ninja 等。现代 CMake 以**目标（target）**为中心：`add_executable/add_library`，用 `target_link_libraries`、`target_include_directories`、`target_compile_features` 并用 `PUBLIC/PRIVATE/INTERFACE` 控制依赖传播。流程：`cmake -S . -B build -DCMAKE_BUILD_TYPE=Release` → `cmake --build build -j`。`find_package(Eigen3 REQUIRED)` 查找依赖。ROS2 的 `ament_cmake` 与 `colcon build` 是其封装。

```cmake
cmake_minimum_required(VERSION 3.16)
project(demo LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_library(ctrl src/pid.cpp)
target_include_directories(ctrl PUBLIC include)
target_link_libraries(ctrl PUBLIC Eigen3::Eigen)

add_executable(demo src/main.cpp)
target_link_libraries(demo PRIVATE ctrl)
```

**English short answer**
CMake is a build system generator: it reads CMakeLists files and produces Makefiles or Ninja files. Modern CMake is target-based, so I declare libraries and executables and link them with PUBLIC, PRIVATE or INTERFACE to control how include paths and dependencies propagate. I configure into a separate build directory, and ROS2 wraps this with ament and colcon.

**追问方向**：PUBLIC/PRIVATE 区别；交叉编译 toolchain file；`find_package` 找不到的排查（`CMAKE_PREFIX_PATH`）。

### Q20 编译优化等级与调试选项？（Compiler flags）
**优先级：★★**

**答案要点**：`-O0`（无优化，便于调试）、`-O1/-O2`（常规发布）、`-O3`（更激进）、`-Os`（优化体积，嵌入式常用）、`-Og`（调试友好优化）；`-g` 生成调试信息；`-Wall -Wextra -Werror` 警告；`-fsanitize=address,undefined,thread`；`-march=native`（只在本机运行时用）；`-DNDEBUG` 关闭 `assert`。嵌入式还有 `-ffunction-sections -fdata-sections -Wl,--gc-sections` 去掉未用代码；`-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard` 指定内核与硬件浮点。

**English short answer**
O0 disables optimization for debugging, O2 is the usual release level, and Os optimizes for size, which is common in embedded work. I always enable strong warnings, and use the sanitizers during development. For a Cortex-M4 I also specify the CPU and hardware floating point options.

**追问方向**：`-O2` 和 `-O3` 的风险；链接脚本与 map 文件。

### Q21 头文件的 include 路径、`#include <>` 与 `""`？（Include paths）
**优先级：★**

**答案要点**：`""` 先从当前文件目录找，再按系统路径；`<>` 直接按 `-I` 和系统路径找。`-I` 增加搜索路径。头文件需 include guard。减少编译时间：前置声明、PIMPL、预编译头。

**English short answer**
Angle brackets search the include paths directly, while quotes first search the directory of the current file. I add paths with the dash-I flag and protect headers with include guards.

**追问方向**：前置声明；循环包含。

### Q22 `undefined reference` / `cannot find -lxxx` / `No such file or directory (so)` 怎么排查？（Link errors）
**优先级：★★★**

**答案要点**
- `undefined reference to foo`：①没链接对应库/源文件（`-lfoo` 缺失）；②链接顺序错（静态库）；③C/C++ 名字改编不匹配，缺 `extern "C"`；④模板定义放在 .cpp；⑤声明了没定义；⑥ABI/标准库版本不一致。
- `cannot find -lfoo`：库路径不在 `-L` 中，或只有运行库没有开发包（`libfoo-dev`）。
- 运行期 `error while loading shared libraries`：`ldd ./app` 看 `not found`，补 `LD_LIBRARY_PATH` / `rpath` / `ldconfig`。
- 工具：`nm -C libfoo.a | grep foo`、`readelf -d app | grep NEEDED`、`ldd`。

**English short answer**
For undefined reference I check that the library or object file is actually linked, that the link order is right, that C++ name mangling matches with extern C where needed, and that templates are defined in headers. For run-time loading errors I use ldd to see what is missing and fix the library path with rpath or ldconfig.

**追问方向**：`nm` 的符号类型；GLIBC 版本不兼容。

### Q23 Docker 与交叉编译在机器人部署中的作用？（Docker and cross-compilation）
**优先级：★**

**答案要点**：Docker 把运行环境（系统库、ROS 发行版）打包成镜像，解决"我这里能跑"；不是虚拟机，共享宿主内核；机器人访问设备需 `--device`、`--network host`、特权设置。交叉编译：在 x86 PC 上用目标架构工具链编译 ARM 程序，再 scp 到板子（RK3588 板端算力有限，常这样做）。

**English short answer**
Docker packages the runtime environment into an image so the program behaves the same everywhere, sharing the host kernel rather than virtualizing hardware. Cross-compilation builds ARM binaries on a faster PC using a target toolchain and then deploys them to the board.

**追问方向**：容器内访问 USB/CAN；多阶段构建。

---

## 四、Git

### Q24 Git 的三个区域和基本工作流？（Git areas and workflow）
**优先级：★★★**

```text
工作区 --git add--> 暂存区(index) --git commit--> 本地仓库 --git push--> 远程仓库
          <--git restore/checkout--                   <--git fetch/pull--
```

**答案要点**：`git status`、`git diff`（工作区 vs 暂存区）、`git diff --staged`、`git add -p`（分块暂存）、`git commit -m`、`git log --oneline --graph`、`git branch`、`git switch -c feat`、`git push -u origin feat`。Git 以**快照**存储，对象有 blob/tree/commit/tag，commit 是指向 tree 和父 commit 的 DAG 节点，分支只是指向 commit 的可移动指针，`HEAD` 指向当前分支。

**English short answer**
Git has a working directory, a staging area and a repository. I stage changes with add, record a snapshot with commit and share with push. Internally a commit points to a snapshot and its parent commits, and a branch is simply a movable pointer to a commit.

**追问方向**：Git 对象模型；SHA-1；`.git` 目录。

### Q25 `merge` 与 `rebase` 的区别？什么时候用？（merge vs rebase）
**优先级：★★★**

**答案要点**
- `merge`：把两个分支历史合并，产生一个**合并提交**（非快进时），保留真实历史，安全。
- `rebase`：把当前分支的提交"摘下来"，在目标分支最新提交之后**重放**，生成新的提交（哈希变化），历史是线性的，更干净。
- 黄金法则：**不要 rebase 已经推送并被他人使用的公共分支**，否则改写历史会导致他人冲突。个人 feature 分支同步主干可用 `git rebase main`；合入主干常用 merge（或 squash merge）。
- `git pull --rebase` 避免无意义的合并提交。

**English short answer**
Merge combines two histories with a merge commit and keeps the true history. Rebase replays my commits on top of the target branch, creating new commits and a linear history. I never rebase commits that others already pulled, since it rewrites history. I rebase my own feature branch onto main, and merge when integrating.

**追问方向**：squash；交互式 rebase `-i` 整理提交；冲突处理。

### Q26 `reset`、`revert`、`checkout/restore` 的区别？（reset vs revert）
**优先级：★★★**

**答案要点**
- `git reset --soft HEAD~1`：撤销提交，保留暂存区和工作区；`--mixed`（默认）：撤销提交和暂存，保留工作区；`--hard`：全部丢弃（危险）。会**改写分支指针**，已推送的不要用。
- `git revert <commit>`：新增一个"反向"提交抵消之前的修改，不改写历史，适合公共分支。
- `git restore file` / `git checkout -- file`：丢弃工作区修改；`git restore --staged file` 取消暂存。
- 误操作救援：`git reflog` 找回 reset 掉的提交。

**English short answer**
Reset moves the branch pointer back and can also change the index and working tree, so it rewrites history and is for local commits only. Revert creates a new commit that undoes an earlier one, so it is safe on shared branches. If I reset by mistake I can usually recover through git reflog.

**追问方向**：`reset --hard` 后如何找回；`git stash`。

### Q27 `git fetch` 与 `git pull`；`origin/main` 是什么？（fetch vs pull）
**优先级：★★★**

**答案要点**：`fetch` 只下载远程更新到远程跟踪分支（`origin/main`），不动工作区；`pull = fetch + merge`（或 `--rebase`）。`origin/main` 是本地对远程状态的缓存引用。`git remote -v` 查看远程；`git push --force-with-lease` 比 `--force` 更安全。

**English short answer**
Fetch downloads new commits and updates the remote-tracking branches without touching my working files. Pull is fetch followed by a merge or rebase. When I must force push, I use force-with-lease, which refuses to overwrite commits I haven't seen.

**追问方向**：detached HEAD；上游分支。

### Q28 怎么解决合并冲突？（Resolving merge conflicts）
**优先级：★★★**

**答案要点**：冲突发生在两个分支修改同一处。文件里会出现 `<<<<<<< HEAD` / `=======` / `>>>>>>> branch` 标记；手动整合成正确内容，删除标记，`git add` 标记已解决，`git merge --continue` / `git rebase --continue`。放弃：`git merge --abort` / `git rebase --abort`。预防：小而频繁的提交、及时同步主干、约定代码格式化、拆分文件。冲突后务必**编译并跑测试**，因为语义冲突不会被标记。

**English short answer**
A conflict appears when both branches changed the same lines. I open the file, combine the changes by hand, remove the markers, stage the file and continue the merge or rebase. Afterwards I always build and test because some semantic conflicts are not detected by Git.

**追问方向**：三方合并原理；mergetool。

### Q29 `cherry-pick`、`stash`、`tag`、`bisect` 的用途？（Other useful Git commands）
**优先级：★★**

**答案要点**：`cherry-pick <commit>`：把某个提交摘到当前分支（修复回补到发布分支）；`stash`：临时存放未提交修改，切换分支后 `stash pop` 恢复；`tag -a v1.0`：给发布点打标签（部署版本追溯）；`bisect`：二分查找引入 bug 的提交（`git bisect start / bad / good`，可 `run` 自动化）；`blame` 看每行最后修改者；`log -S"关键字"` 查字符串何时出现。

**English short answer**
Cherry-pick copies a single commit onto the current branch, stash shelves uncommitted work temporarily, tags mark release points, and bisect performs a binary search through the history to find the commit that introduced a bug.

**追问方向**：bisect 的复杂度 $O(\log n)$；`blame` 误用。

### Q30 `.gitignore`、大文件、子模块、LFS？（gitignore, large files, submodules）
**优先级：★★**

**答案要点**：`.gitignore` 忽略编译产物、日志、IDE 配置、密钥（已被跟踪的文件需先 `git rm --cached`）。**不要提交**：二进制大文件（仿真输出、模型权重）、密钥与令牌；大文件用 Git LFS 或外部存储。子模块 `git submodule add/update --init --recursive` 引用别的仓库的固定提交；如果子模块目录为空，说明没 `--recursive` 克隆或远程不可访问。已误提交的敏感信息要轮换令牌，并用 `git filter-repo` 清理历史。

**English short answer**
The gitignore file keeps build outputs, logs and secrets out of the repository. Large binaries such as model weights belong in Git LFS or external storage. A submodule pins another repository at a specific commit, and if its folder is empty I have probably forgotten to initialize it recursively. Any secret committed by mistake must be rotated, not just deleted.

**追问方向**：子模块 vs 子树；monorepo。

### Q31 分支策略与代码评审流程？（Branching strategies and review）
**优先级：★★**

**答案要点**：主流：①Git Flow（main/develop/feature/release/hotfix），适合版本发布；②GitHub Flow / trunk-based：短寿命 feature 分支 + Pull Request + CI，合入 main。好的提交信息：一句话概括（祈使句）+ 必要时空行后说明原因；一次提交只做一件事；PR 小而可评审；CI 跑编译、静态检查、单元测试。机器人固件/算法仓库常见做法：参数与代码分开管理，发布时打 tag 并记录对应硬件版本。

**English short answer**
Common strategies are Git Flow with long-lived develop and release branches, and a simpler trunk-based flow with short feature branches and pull requests checked by continuous integration. I keep commits focused with clear messages, and keep pull requests small so they are easy to review.

**追问方向**：Conventional Commits；CI 配置。

### Q32 Git 面试高频追问：`HEAD`、`detached HEAD`、`ORIG_HEAD`、`reflog` 是什么？（HEAD concepts）
**优先级：★★**

**答案要点**：`HEAD` 是当前检出位置（通常指向某分支）；`detached HEAD`：直接检出某个提交/标签，此时新提交不属于任何分支，切走后容易丢，需 `git switch -c new` 保存；`reflog` 记录 HEAD 的移动历史，是找回丢失提交的保险；`HEAD~1`/`HEAD^` 上一个提交。

**English short answer**
HEAD points to what I currently have checked out, usually a branch. In a detached HEAD state I am on a specific commit, and new commits would be lost unless I create a branch. The reflog records where HEAD has been, so I can recover commits that look lost.

**追问方向**：`git switch -`；找回已删除分支。

---

## 五、综合场景

### Q33 ROS2 工作空间怎么编译？常见编译问题？（colcon build）
**优先级：★★★**

**答案要点**：`colcon build --symlink-install --packages-select pkg`；`source install/setup.bash`；CMake 包用 `ament_cmake`，Python 包用 `ament_python`；依赖在 `package.xml` 声明，`rosdep install --from-paths src --ignore-src -r -y`。常见问题：找不到包 → 没 source；消息头文件找不到 → 依赖未声明或未先生成接口；改了 Python 文件没生效 → 没用 `--symlink-install`；多工作空间覆盖顺序；清理 `rm -rf build install log`。详见 [ROS1_ROS2核心概念](../../02_基础知识/09_机器人软件/04_ROS1_ROS2核心概念.md)。

**English short answer**
I build with colcon, using symlink install so Python edits take effect, then source the install setup script. Typical problems are forgetting to source the workspace, missing dependency declarations in package.xml, and stale build directories, which I fix with a clean rebuild.

**追问方向**：overlay/underlay；`ament_target_dependencies`。

### Q34 板端（RK3588 / 嵌入式 Linux）常见部署流程？（Deploying to an embedded Linux board）
**优先级：★★**

**答案要点**：典型做法：①交叉编译或在板子上编译；②`scp/rsync` 部署；③systemd 服务开机自启；④日志用 journald 并限制大小；⑤设备权限用 udev 规则；⑥版本管理：打 tag，记录依赖库版本；⑦远程调试 ssh + gdbserver；⑧只读根文件系统与 OTA 升级（进阶）。与 P4 的 RK3588 边缘推理部署相关：模型转换（RKNN）在 PC 上完成，板端只跑推理运行时（见 CV 的"YOLOv5/RKNN INT8 inference chain on RK3588"）。我们的做法是：RKNN 模型转换在 PC 上完成；板端运行的 Python 推理脚本与 RKNN 运行时直接在 RK3588 上执行，不需要交叉编译；用 `rsync` 同步脚本与模型；用 systemd 服务实现开机自启与异常重启（`Restart=on-failure`），更新时先同步文件再 `systemctl restart`；Hi3861 端则在 PC 上用 OpenHarmony 构建工具链交叉编译后烧录。

**English short answer**
I either cross-compile on a PC or build on the board, copy the binaries with rsync, run them as a systemd service that starts at boot, and manage device permissions with udev rules. For the RK3588 vision pipeline the model conversion is done on the PC and only the inference runtime runs on the board.

**追问方向**：OTA；日志轮转；看门狗。

### Q35 你的笔记本/开发机环境与机器人上不一致怎么办？（Environment consistency）
**优先级：★★**

**答案要点**：固定工具链和依赖版本（Dockerfile、`requirements.txt`、`rosdep`）；在同架构上测试；CI 在容器里构建；用仿真（MuJoCo）先验证逻辑；`uname -m` 确认架构，`ldd --version` 确认 glibc；避免依赖本机路径；配置与参数外置（YAML）。

**English short answer**
I pin the toolchain and dependency versions, use containers for reproducible builds, and verify the architecture and library versions on the target. I keep parameters in config files rather than hard-coded paths, and test logic in simulation before touching hardware.

**追问方向**：glibc 版本；ABI。

### Q36 systemd 服务怎么写？怎么看日志？（systemd and journalctl）
**优先级：★★**

**答案要点**
- systemd 是现代 Linux 的初始化系统与服务管理器。服务单元文件放 `/etc/systemd/system/xxx.service`，核心三段：`[Unit]`（描述、`After=network-online.target` 依赖顺序）、`[Service]`（`ExecStart`、`User`、`Restart=on-failure`、`RestartSec`、`Environment`）、`[Install]`（`WantedBy=multi-user.target`）。
- 常用命令：`systemctl daemon-reload`（改了单元文件后）、`enable --now`、`status`、`restart`、`stop`；日志：`journalctl -u robot.service -f`（跟随）、`-b`（本次启动）、`--since "10 min ago"`、`-p err`。
- 机器人上的注意点：①服务里没有交互终端环境，`source setup.bash` 的环境变量要在 `Environment=` 或包装脚本里设置；②设备（串口、CAN）未就绪时用 `After=`/`Restart` 兜底；③限制日志体积 `SystemMaxUse=`（`journald.conf`）；④`WatchdogSec=` 可与程序的心跳配合，卡死自动重启。

**English short answer**
I write a systemd unit with an ExecStart command, a restart policy and a WantedBy target, then enable it so it starts at boot and restarts on failure. I read its logs with journalctl following the unit. On a robot I pay attention to the environment variables, the start order relative to devices and network, and a size limit on the journal.
（中文：写一个带 ExecStart、重启策略和 WantedBy 的 systemd 单元并 enable，实现开机自启和失败重启；用 journalctl 跟随查看日志；机器人上要注意环境变量、相对设备和网络的启动顺序、以及日志体积上限。）

**追问方向**：`Type=simple/forking/notify`；用户级服务；看门狗集成。

### Q37 SocketCAN 是什么？怎么在 Linux 上调试 CAN 总线？（SocketCAN）
**优先级：★★**

**答案要点**
- SocketCAN 把 CAN 控制器抽象成网络接口（`can0`），用 socket API（`socket(PF_CAN, SOCK_RAW, CAN_RAW)`）收发帧，可被多个进程同时读取。ROS/ROS2 的 CAN 电机驱动（与 CV 中 P1/P2 的 "ROS state processing with CAN motor execution" 同类架构）常通过它与 MCU 或电机通信。
- 配置（需 root）：`ip link set can0 type can bitrate 1000000`，`ip link set up can0`；CAN FD 另设 `dbitrate` 并 `fd on`。
- 调试工具（`can-utils`）：`candump can0`（抓帧）、`cansend can0 123#DEADBEEF`（发帧）、`cangen`（压力测试）、`canbusload`（总线负载）；`ip -details -statistics link show can0` 查看错误计数与 bus-off 状态。
- 常见问题：波特率不一致 → 大量错误帧；终端电阻缺失（两端各 120 Ω）；总线 bus-off 需重启接口（`restart-ms`）；发送队列满（`txqueuelen` 过小）。典型波特率 500 kbit/s～1 Mbit/s（典型取值，以实机为准）。

**English short answer**
SocketCAN exposes a CAN controller as a network interface, so I can use sockets and standard tools. I set the bit rate with ip link, watch traffic with candump, send test frames with cansend, and check error counters and bus load. Typical problems are mismatched bit rates, missing termination resistors, and a bus-off state.
（中文：SocketCAN 把 CAN 控制器当网络接口，可用 socket 和标准工具；用 ip link 设置波特率，candump 抓帧，cansend 发测试帧，并检查错误计数与总线负载；常见问题是波特率不匹配、缺终端电阻、bus-off。）

**追问方向**：CAN 仲裁与 ID 优先级；错误帧；CAN FD。

### Q38 core dump 与 `ulimit`：怎么事后分析崩溃？（Core dumps and ulimit）
**优先级：★★**

**答案要点**
- 程序崩溃（段错误、`abort`）时可由内核生成 core 文件，记录当时的内存与寄存器。步骤：①编译带 `-g`（最好 `-O0` 或 `-Og`）；②`ulimit -c unlimited` 允许生成；③`cat /proc/sys/kernel/core_pattern` 查看去向（现代发行版可能交给 `systemd-coredump`，用 `coredumpctl list/gdb` 查看）；④`gdb ./prog core`，`bt` 看调用栈，`info threads`、`thread apply all bt` 看所有线程。
- `ulimit` 管理进程资源限制：`-n` 打开文件数、`-s` 栈大小、`-c` core 大小、`-l` 锁定内存（实时程序 `mlockall` 需要）、`-r` 实时优先级；永久修改写 `/etc/security/limits.conf` 或 systemd 的 `LimitCORE=` 等。
- 没有调试符号时：保留带符号版本，用 `addr2line` 把地址转为行号；AddressSanitizer（`-fsanitize=address`）能在崩溃前给出更准确的越界/释放后使用信息。

**English short answer**
To analyse a crash after the fact I build with debug symbols, allow core files with ulimit, and open the core in gdb to print the backtrace of all threads. On systemd machines I use coredumpctl. If the binary has no symbols, I use addr2line on a symbolized build, and AddressSanitizer gives more precise reports for memory errors.
（中文：事后分析崩溃：带调试符号编译，用 ulimit 允许生成 core，在 gdb 中查看所有线程的回溯；systemd 机器用 coredumpctl；无符号时用 addr2line；AddressSanitizer 对内存错误给出更精确的报告。）

**追问方向**：`core_pattern`；Release 与 Debug 符号分离；Valgrind。

### Q39 `free -h` 怎么看？buffers/cache、swap 与 OOM killer？（Memory in Linux）
**优先级：★★**

**答案要点**
- `free -h` 中重点看 **available**（可供新进程使用的内存，含可回收的页缓存），而不是 `free`；Linux 会把空闲内存用作页缓存（`buff/cache`），所以 `free` 小是正常的。
- `swap`：把冷页换到磁盘；机器人嵌入式板常关闭或很小，因为换页会造成不可预测的延迟；`vmstat 1` 里 `si/so` 持续非零说明在换页。
- OOM killer：内存耗尽时内核按 `oom_score` 杀进程；`dmesg | grep -i "killed process"` 确认；可通过 `oom_score_adj` 保护关键进程，或用 cgroup/容器内存限制把风险隔离在容器内。
- 排查泄漏：`top`/`ps` 看 `RSS` 是否单调上涨；`pmap`、`smem`；用户态用 Valgrind massif、heaptrack 或 ASan 的 LeakSanitizer；Python 用 `tracemalloc`。
- 嵌入式板（如 RK3588）上，GPU/NPU 的内存常与系统共享，要把模型与图像缓冲算进预算。

**English short answer**
I look at the available column rather than free, because Linux uses spare memory as page cache and reclaims it when needed. Constant swap activity is a warning sign, and on a robot I try to avoid swapping since it adds unpredictable latency. If a process disappears, I check dmesg for the OOM killer, and to find leaks I watch the resident size over time and use a heap profiler.
（中文：看 available 而非 free，因为 Linux 会把空闲内存用作页缓存并按需回收；持续换页是警告信号，机器人上要避免换页带来的不可预测延迟；进程消失时在 dmesg 里查 OOM killer；排查泄漏时观察驻留内存趋势并用堆分析器。）

**追问方向**：`/proc/meminfo`；cgroup 内存限制；HugePages。

### Q40 CMake 如何做交叉编译？toolchain 文件是什么？（CMake cross-compilation）
**优先级：★★**

**答案要点**
- 交叉编译：在 x86 开发机上编译出 ARM（如 aarch64 的 RK3588）可执行文件。需要：交叉编译器（如 `aarch64-linux-gnu-g++`）、目标系统的 **sysroot**（目标板的头文件与库）。
- CMake 用 **toolchain 文件**描述：`CMAKE_SYSTEM_NAME Linux`、`CMAKE_SYSTEM_PROCESSOR aarch64`、`CMAKE_C_COMPILER`/`CMAKE_CXX_COMPILER`、`CMAKE_SYSROOT`、`CMAKE_FIND_ROOT_PATH` 以及 `CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER`（程序在宿主找）、`..._LIBRARY/INCLUDE ONLY`（库与头文件在目标 sysroot 找）。
- 使用：`cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=aarch64.cmake`，然后 `cmake --build build`，产物用 `file` 命令确认架构。
- 常见坑：链接了宿主机的 `.so`（架构不符）；glibc 版本比目标板新导致运行时报 `GLIBC_x.y not found`；`pkg-config` 要设置 `PKG_CONFIG_SYSROOT_DIR`/`PKG_CONFIG_LIBDIR`。Docker 里放工具链可保证可复现（见 Q23）。
- 与 CV 相关：P4 的 Hi3861 端在 PC 上用 OpenHarmony 构建工具链交叉编译后烧录。

```cmake
# 教学示例: aarch64.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /opt/sysroot-rk3588)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

**English short answer**
For cross-compiling with CMake I write a toolchain file that sets the target system, the cross compiler and the sysroot of the target board, and tells CMake to find libraries and headers in the sysroot but programs on the host. I then pass the file at configure time and check the output with the file command. The usual pitfalls are linking host libraries and a glibc version mismatch with the board.
（中文：用 CMake 交叉编译时写 toolchain 文件，指定目标系统、交叉编译器和目标板 sysroot，并让库和头文件在 sysroot 中查找、程序在宿主机查找；配置时传入该文件，并用 file 命令检查产物；常见坑是链接了宿主机库和 glibc 版本与目标板不匹配。）

**追问方向**：sysroot 如何制作；`qemu-user` 运行测试；交叉产物上不能直接用 `ldd`，改用 `readelf -d`。

---

## 本章要点速记
1. 排障先定位：CPU（top/perf）、内存（free available/OOM）、磁盘（df/du/lsof）、网络（ss/ping/tcpdump）。
2. 信号处理函数里只置标志；机器人程序退出前先使能失效/失能电机。
3. `2>&1` 的顺序；`set -euo pipefail`；变量加引号。
4. 权限 rwx = 421；设备权限靠用户组/udev。
5. 编译四阶段：预处理 → 编译 → 汇编 → 链接；`undefined reference` 是链接错误。
6. 静态库拷进可执行文件，动态库运行时加载，注意路径与版本。
7. CMake 以 target 为中心，PUBLIC/PRIVATE 控制传播。
8. rebase 不用于已共享分支；reset 改历史，revert 加反向提交；reflog 救命。
9. 不要提交大文件和密钥；子模块要 `--recursive`。

## 自测题
1. `ps` 中 D 状态进程是什么？为什么 `kill -9` 杀不掉？
2. `cmd > f 2>&1` 与 `cmd 2>&1 > f` 的区别？
3. 写出编译 `main.c` 并链接静态库 `libfoo.a` 的命令。
4. 运行时报 `libxxx.so: cannot open shared object file`，怎么排查？
5. `merge` 和 `rebase` 各自的优缺点？什么情况下不能 rebase？
6. 误执行 `git reset --hard` 后如何找回提交？

## 相关链接
- 上一章：[并发多线程与实时系统八股](./07_并发多线程与实时系统八股.md)　下一章：[数据库与系统设计基础](./09_数据库与系统设计基础.md)
- 相关：[Linux与Shell_Git](../../02_基础知识/09_机器人软件/01_Linux与Shell_Git.md)、[八股总入口](../README.md)
