# 嵌入式 Linux：启动流程、设备树、驱动模型、PREEMPT_RT 与时延预算

> **一句话定位**：RK3588 这类"小电脑"跑的是 Linux（P4 的边缘视觉、P1/P2 的 ROS 2 节点与 SocketCAN 都在 Linux 上）。这一章讲嵌入式 Linux 与裸机/RTOS 的根本区别：硬件怎样被设备树描述、驱动怎样与硬件绑定并以 `/dev` 节点暴露给用户态、用户态与内核态的边界怎样影响时延，以及用 PREEMPT_RT 与 SCHED_FIFO 能把时延压到什么程度——并诚实标出"哪些是 CV 项目里用到的，哪些是延伸知识"。

**本章学完你能：**

- 画出嵌入式 Linux 从上电到用户空间的启动链（BootROM、SPL、U-Boot、内核、设备树、根文件系统、init）；
- 读懂一个设备树节点（`compatible`、`reg`、`interrupts`、`clocks`、`status`、`pinctrl`），说清它与驱动的匹配过程；
- 写出一个最小的字符设备/平台驱动骨架，解释 `copy_to_user`、`ioctl`、`file_operations`；
- 区分上半部/下半部（top half/bottom half），说出线程化中断的意义；
- 解释 PREEMPT_RT 做了什么，在用户态用 SCHED_FIFO、`mlockall`、`clock_nanosleep` 做一个像样的周期任务；
- 用 cyclictest 的统计数据判断系统实时性，并做时延预算的手算；
- 说清 Linux 与 RTOS/MCU 各自适合做什么，为什么机器人常"上位机 Linux + 下位机 MCU"；
- 用英文回答 "Can you do real-time control on Linux?"

**前置知识：**[OpenHarmony、Hi3861、RK3588 平台基础](./06_OpenHarmony_Hi3861_RK3588平台基础.md)（两类内核对比、V4L2、NPU）；[Linux 与 Shell、Git](../12_机器人软件/01_Linux与Shell_Git.md)；[实时调度理论](./11_实时调度理论_RMS_EDF_优先级反转.md)；C 语言指针与结构体。

**相关项目：**[P4 平台基础](../../01_项目课程/P4_OpenHarmony智能园艺系统/02_平台基础_OpenHarmony_Hi3861_RK3588.md)、[P4 RK3588 边缘视觉：V4L2 取帧](../../01_项目课程/P4_OpenHarmony智能园艺系统/05_RK3588边缘视觉_V4L2取帧与预处理.md)、[P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md)、[P6 OpenHarmony 基础](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md)。

## 目录

1. 要解决什么问题
2. 符号与术语
3. 启动链
4. 设备树
5. 驱动模型与最小驱动
6. 中断：上半部与下半部
7. 用户态与内核态的时延
8. PREEMPT_RT 与用户态实时编程
9. 时延预算与统计的完整手算
10. 在七个项目里怎么用
11. 常见错误与故障现象
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

### 1.1 为什么 MCU 之外还要 Linux

MCU（STM32）擅长**确定性**：微秒级的中断响应、固定周期的控制。但它的内存只有几百 KB 到几 MB，没有虚拟内存和文件系统，难以跑视觉模型、ROS 2、Web 服务、数据库。RK3588 一类 SoC 有数 GB 内存、NPU、摄像头接口，能跑完整 Linux，擅长**算力与生态**，但时延**不确定**。机器人系统因此常分层：

```text
  上位机 (Linux, RK3588 / x86 / Jetson)      下位机 (MCU + RTOS)
  视觉推理、规划、ROS 2、网络、UI   <--CAN/UART/以太网-->  电机控制环、传感器采样、安全保护
  吞吐大、时延抖动 (ms 级)                                  时延小、抖动 (µs 级)
```

P3 的 STM32 + PC 视觉、P7 的 H7 + 上位机、P4 的 RK3588 + Hi3861 都是这个分层。理解 Linux 一侧的行为——设备怎么出现、驱动怎么工作、时延从哪来——才能回答"你的视觉到控制整条链的延迟是怎么来的"。

### 1.2 本章的三个核心问题

1. **硬件怎么告诉内核**：设备树；
2. **内核怎么把硬件交给用户态**：驱动与设备节点；
3. **能不能做实时**：PREEMPT_RT + 正确的用户态写法 + 测量。

---

## 2. 符号与术语

| 术语 | 含义 |
|---|---|
| BootROM | 芯片内固化的第一段代码 |
| SPL / TPL | 二级/三级引导程序（初始化 DDR 内存等），RK 平台常称 idbloader |
| U-Boot | 常用引导加载器，加载内核、设备树 |
| DTS / DTB / DTC | 设备树源文件 / 二进制 / 编译器 |
| `compatible` | 设备树节点的"型号字符串"，驱动用它匹配 |
| IRQ | 中断请求 |
| Top half / bottom half | 中断处理的上半部（硬中断）/ 下半部（软中断、tasklet、工作队列、线程化中断） |
| WCET | 最坏执行时间 |
| $L_{irq}$ | 中断延迟：硬件事件到 ISR 开始执行 |
| $L_{sched}$ | 调度延迟：线程可运行到真正在 CPU 上运行 |
| $L_{total}$ | 总时延：硬件事件到用户态响应 |
| jitter | 周期抖动：实际周期与标称周期之差 |

时间单位用 µs 与 ms。

---

## 3. 启动链

### 3.1 一般流程

```text
上电 -> BootROM (芯片内)
     -> 读取存储介质上的第一阶段引导 (SPL / idbloader)，初始化 DDR
     -> (可选) 安全固件，如 ARM Trusted Firmware (BL31)
     -> U-Boot：初始化更多外设，加载 kernel Image、设备树 DTB、(initramfs)
     -> Linux 内核：解压/初始化，解析 DTB，探测并绑定驱动，挂载根文件系统
     -> init (systemd / busybox init / OpenHarmony init)
     -> 用户空间服务，应用
```

要点：

- **每一级只做下一级需要的最少工作**。BootROM 空间小，只能识别少数存储介质；DDR 初始化代码（SPL）必须在 SRAM 中运行，因为这时 DDR 还不可用；
- U-Boot 通过**内核命令行**（`bootargs`）传递根文件系统位置、控制台串口、`isolcpus` 等参数；
- OpenHarmony 标准系统在 RK3588 上使用 Linux 内核，用户空间由 OpenHarmony 的 init 与服务框架接管（见 [平台基础](./06_OpenHarmony_Hi3861_RK3588平台基础.md) 与 [P6 OpenHarmony 基础](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md)）。

> **诚实边界**：具体到 RK3588 的分区、烧写格式与厂商 SDK 流程，本章没有逐项核对原厂文档；面试只需要掌握上述通用链条。

### 3.2 与 MCU 启动的对比

| 项 | MCU（Cortex-M） | 嵌入式 Linux |
|---|---|---|
| 启动 | 硬件读向量表，直接跑固件（见 [07 章](./07_Cortex-M体系_启动_链接脚本_内存映射.md)） | 多级引导，加载内核与设备树 |
| 硬件描述 | 写死在代码里（寄存器地址、引脚） | 设备树描述，驱动通用 |
| 内存 | 物理地址，无 MMU | 虚拟内存，MMU，进程隔离 |
| 出错后果 | HardFault，整机可能死 | 进程崩溃，系统通常继续运行（内核崩溃 oops/panic 除外） |

---

## 4. 设备树

### 4.1 要解决什么问题

早期 ARM Linux 内核里，每块板子的硬件信息（外设基地址、中断号、引脚）都用 C 代码写在内核里（board file），内核源码里堆满了板级代码，难以维护。设备树把**硬件描述从内核代码里分离出来**，成为一个独立的数据文件：同一份内核二进制，换一个设备树就能启动不同的板子。

### 4.2 语法与示例

设备树是一棵节点树，节点由属性（键值对）与子节点组成。示例（**教学示例**，节点名与属性为演示，非某板子的真实配置）：

```text
/ {
    model = "demo-board";
    compatible = "vendor,demo-board";
    #address-cells = <1>;
    #size-cells = <1>;

    soc {
        #address-cells = <1>;
        #size-cells = <1>;
        ranges;

        spi1: spi@40013000 {
            compatible = "vendor,demo-spi";
            reg = <0x40013000 0x400>;          /* 基地址和大小 */
            interrupts = <0 35 4>;             /* 中断控制器相关的编码 */
            clocks = <&cru 120>;               /* 引用时钟控制器 */
            status = "okay";
            #address-cells = <1>;
            #size-cells = <0>;

            imu@0 {
                compatible = "bosch,bmi088-gyro";   /* 示例：IMU 的陀螺部分 */
                reg = <0>;                          /* 片选号 */
                spi-max-frequency = <5000000>;
                interrupt-parent = <&gpio3>;
                interrupts = <12 2>;                /* GPIO 3 的第 12 脚，边沿触发 */
            };
        };
    };
};
```

读法：

- `reg`：外设寄存器的地址与长度，其单位由父节点的 `#address-cells`、`#size-cells` 决定；
- `compatible`：字符串列表，从最具体到最通用；内核用它来匹配驱动；
- `status = "okay"` 启用，`"disabled"` 禁用（便于板级文件覆盖）；
- `clocks`、`pinctrl-0`、`interrupts`：引用其他节点（phandle，`&label`）；
- 子节点：挂在总线下的设备，如 SPI 控制器下的从设备，`reg` 是片选号。

### 4.3 编译与加载

`dtc -I dts -O dtb -o board.dtb board.dts`；U-Boot 把 DTB 加载到内存，内核启动时通过寄存器传入 DTB 地址。**设备树叠加（overlay）** 允许在运行时或启动时修改部分节点，如为扩展板启用某个 SPI 设备，而不需要重编整个 DTB。

### 4.4 设备树与驱动的匹配

内核启动时：

1. 解析 DTB，为每个"可用"的节点创建 `platform_device`（总线上的子设备由总线驱动创建，如 SPI 控制器驱动为 `imu@0` 创建 `spi_device`）；
2. 驱动注册时给出一张 `of_device_id` 表（`compatible` 字符串列表）；
3. 总线核心比较两者，匹配成功则调用驱动的 `probe()`，传入设备，驱动从设备树取属性（`of_property_read_u32` 等）、申请资源（`devm_ioremap_resource`、`devm_request_irq`）、注册设备节点。

**调试经验**：设备没出现，先看（1）`status` 是否 okay；（2）`compatible` 字符串是否与驱动一致；（3）引脚复用（pinctrl）有没有被别的功能占用；（4）`dmesg | grep -i <设备>` 里有没有 probe 失败信息（常见 `-EPROBE_DEFER` 表示依赖的时钟/GPIO 驱动还没就绪，稍后重试）。

---

## 5. 驱动模型与最小驱动

### 5.1 用户态与内核态

CPU 有特权级别：内核态可访问全部硬件和内存，用户态受限。用户程序通过**系统调用**（`open`、`read`、`write`、`ioctl`、`mmap`）进入内核。每次系统调用有开销（量级：几百纳秒到数微秒，取决于 CPU 与缓解措施；本章不给定值），因此高频控制中应减少系统调用次数。

### 5.2 Linux 驱动的类型

| 类型 | 例子 | 用户态接口 |
|---|---|---|
| 字符设备 | 串口、I2C/SPI 设备、摄像头（V4L2）、SocketCAN 之外的自定义设备 | `/dev/xxx` |
| 块设备 | eMMC、SD 卡 | 文件系统 |
| 网络设备 | 以太网、**CAN（SocketCAN）** | socket，`ip link` |

SocketCAN 把 CAN 控制器当作**网络设备**（`can0`），用户态用 `socket(PF_CAN, ...)` 收发帧，这就是 P2 的 ROS 2-CAN 执行链 在 Linux 侧的实现（见 [P2 ROS2-CAN 执行链](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/10_ROS2_CAN执行链.md) 的 SocketCAN 配置命令）。

### 5.3 最小平台驱动骨架（教学示例）

下面的驱动是示意性的"读一个寄存器并通过 ioctl 返回"，**不是项目源码**。

```c
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/uaccess.h>

struct demo_dev {
    void __iomem *base;
    struct miscdevice misc;
};

static ssize_t demo_read(struct file *f, char __user *buf, size_t len, loff_t *off)
{
    struct demo_dev *d = container_of(f->private_data, struct demo_dev, misc);
    u32 v = readl(d->base + 0x10);              /* 读寄存器 */
    if (len < sizeof(v)) return -EINVAL;
    if (copy_to_user(buf, &v, sizeof(v)))       /* 内核 -> 用户，必须用此函数 */
        return -EFAULT;
    return sizeof(v);
}

static const struct file_operations demo_fops = {
    .owner = THIS_MODULE,
    .read  = demo_read,
};

static int demo_probe(struct platform_device *pdev)
{
    struct demo_dev *d;
    struct resource *res;

    d = devm_kzalloc(&pdev->dev, sizeof(*d), GFP_KERNEL);
    if (!d) return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);   /* 来自设备树 reg */
    d->base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(d->base)) return PTR_ERR(d->base);

    d->misc.minor = MISC_DYNAMIC_MINOR;
    d->misc.name  = "demo0";                     /* /dev/demo0 */
    d->misc.fops  = &demo_fops;
    platform_set_drvdata(pdev, d);
    return misc_register(&d->misc);
}

static int demo_remove(struct platform_device *pdev)
{
    struct demo_dev *d = platform_get_drvdata(pdev);
    misc_deregister(&d->misc);
    return 0;
}

static const struct of_device_id demo_of_match[] = {
    { .compatible = "vendor,demo-sensor" },
    { }
};
MODULE_DEVICE_TABLE(of, demo_of_match);

static struct platform_driver demo_driver = {
    .probe  = demo_probe,
    .remove = demo_remove,
    .driver = { .name = "demo", .of_match_table = demo_of_match },
};
module_platform_driver(demo_driver);
MODULE_LICENSE("GPL");
```

关键概念：

- `devm_*`：设备管理资源，驱动卸载或 probe 失败时自动释放，减少泄漏；
- `copy_to_user / copy_from_user`：内核不能直接解引用用户指针（可能无效、可能被换出），必须用这些函数，失败返回非零；
- `readl / writel`：带内存屏障的寄存器访问；不要用普通指针读写寄存器（编译器可能重排或省略）；
- `container_of`：由成员指针得到包含它的结构体指针；
- `ioremap`：把物理地址映射到内核虚拟地址。MMU 存在，所以不能把物理地址当指针用。

### 5.4 用户态访问方式

| 方式 | 说明 | 时延特点 |
|---|---|---|
| `read/write/ioctl` | 经典系统调用 | 每次进内核，适合中低频 |
| `mmap` | 把设备内存或内核缓冲映射到用户空间 | 数据零拷贝（V4L2 取帧就用 `mmap` 缓冲，见 [P4 V4L2](../../01_项目课程/P4_OpenHarmony智能园艺系统/05_RK3588边缘视觉_V4L2取帧与预处理.md)） |
| `poll/epoll/select` | 等待事件 | 配合非阻塞 I/O |
| sysfs / debugfs | 属性文件 | 配置、调试，不用于高频 |
| 用户态驱动（UIO、`/dev/mem`、libgpiod） | 绕开内核驱动 | 快速原型；`/dev/mem` 直接访问寄存器要极小心 |

---

## 6. 中断：上半部与下半部

### 6.1 为什么要拆分

硬中断处理（上半部）运行时，本 CPU 上同级及更低优先级的中断可能被屏蔽，必须极短。因此中断处理分两段：

- **上半部**：确认中断、读取状态、清除中断源、记录时间戳，调度下半部；
- **下半部**：耗时处理，可被抢占：softirq、tasklet、工作队列（workqueue）、**线程化中断（threaded IRQ）**。

线程化中断：`request_threaded_irq(irq, hard_handler, thread_fn, flags, name, dev)`。`thread_fn` 在内核线程中运行，有优先级，可被调度器管理，也可以睡眠（获取互斥锁）。**PREEMPT_RT 把几乎所有中断处理都强制线程化**，这样中断处理也能被更高优先级任务抢占，是它实现低时延的关键之一。

### 6.2 时间线（以一次 GPIO 中断唤醒用户线程为例）

```text
硬件事件 (GPIO 边沿)
   |  L_irq  : 中断控制器路由、CPU 响应、保存现场
   v
上半部 ISR (记时间戳, 清中断)
   |  唤醒等待线程
   v
调度器选择线程运行
   |  L_sched: 线程可运行 -> 在 CPU 上运行
   v
用户线程 (read/poll 返回) -> 处理 -> 写设备
```

总时延 $L_{total}=L_{irq}+L_{isr}+L_{sched}+L_{user}+L_{out}$。在非实时内核里，$L_{sched}$ 的最坏值可能是毫秒级（被长的内核临界区、其他中断、页错误拖延）；在 PREEMPT_RT 上可降到几十至一两百微秒量级（取决于硬件，**需要自己的 cyclictest 实测**）。

---

## 7. 用户态与内核态的时延

### 7.1 来源清单

| 来源 | 说明 |
|---|---|
| 调度延迟 | 线程就绪后等待 CPU；与优先级、抢占点密度有关 |
| 内核不可抢占区 | 自旋锁持有期间、关中断区间、长的内核路径 |
| 页错误 | 访问未驻留的页（首次触碰、被换出、内存映射文件）→ 阻塞等待 I/O |
| 缓存/TLB 缺失 | 上下文切换后冷缓存，执行时间增加 |
| CPU 频率调节 | 省电策略降频，唤醒后频率爬升；实时系统应固定为性能模式 |
| SMI/固件中断（x86） | 对软件不可见的时间窃取；ARM 板上较少，但电源管理固件可能存在 |
| 竞争与优先级反转 | 内核内部与用户态的锁 |
| 其他中断与软中断 | 网络收包、存储、USB 等 |
| 热节流 | 过热时降频（RK3588 在高负载下需要散热） |

### 7.2 为什么 ROS 2 在 Linux 上也能跑机器人

ROS 2 节点在用户态，时延抖动在毫秒量级；对 100 Hz 的规划/估计、20–50 Hz 的遥操作，毫秒抖动没问题。而 500 Hz–1 kHz 的力矩控制需要的确定性，就交给 MCU。这正是 P7（H7 上跑 500 Hz 控制）与 P2（ROS 2 节点经 SocketCAN 发命令、底盘 MCU 内部做轮速闭环）的分工。CV 里写的是 "validated on real hardware through the ROS2-CAN execution chain"，没有写你把 Linux 调成硬实时。

---

## 8. PREEMPT_RT 与用户态实时编程

### 8.1 内核抢占模型

Linux 内核配置里的抢占选项：

| 选项 | 含义 | 典型用途 |
|---|---|---|
| `PREEMPT_NONE` | 内核态不可被抢占（只在系统调用返回等点调度） | 服务器，吞吐优先 |
| `PREEMPT_VOLUNTARY` | 加入显式的让出点 | 桌面 |
| `PREEMPT` (low-latency desktop) | 内核态除少数临界区外可抢占 | 低时延桌面 |
| `PREEMPT_RT` | 几乎所有内核代码可抢占；自旋锁变成可睡眠的实时互斥锁（带优先级继承）；中断线程化；高精度定时器 | 软/准硬实时控制 |

PREEMPT_RT 最初是一组外部补丁（RT-Preempt patch）；据公开报道，它在 2024 年合并进主线内核（约在 6.12 版本，具体版本以内核发布说明为准，本文未核对）。使用时要确认厂商 BSP 的内核版本和是否提供 RT 补丁——RK3588 的厂商内核常较旧，RT 补丁的可用性要查证（**未核对**）。

### 8.2 PREEMPT_RT 做了什么（概念）

1. **自旋锁→rt_mutex**：持锁期间可被抢占，避免长时间关抢占；rt_mutex 带优先级继承，直接对应 [实时调度理论](./11_实时调度理论_RMS_EDF_优先级反转.md) 里的协议；
2. **中断线程化**：硬中断只做最少工作，主体在内核线程里按优先级调度；
3. **高分辨率定时器 (hrtimer)**：定时精度到微秒甚至更细，而不是依赖 jiffies（tick）；
4. **减少不可抢占段**：把大的临界区拆小；
5. **实时限流（RT throttling）**：默认 `sched_rt_runtime_us=950000`、`sched_rt_period_us=1000000`，即实时任务每秒最多占 95% CPU，保护系统不被失控的实时线程锁死；这是 Linux 的**安全网**，实时线程出现死循环时仍留 5% 给其他任务。

### 8.3 用户态写一个像样的周期任务

```c
#define _GNU_SOURCE
#include <sched.h>
#include <time.h>
#include <sys/mman.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#define PERIOD_NS 2000000L            /* 2 ms = 500 Hz */

static void add_ns(struct timespec *t, long ns)
{
    t->tv_nsec += ns;
    while (t->tv_nsec >= 1000000000L) { t->tv_nsec -= 1000000000L; t->tv_sec++; }
}

int main(void)
{
    struct sched_param sp = { .sched_priority = 80 };
    if (sched_setscheduler(0, SCHED_FIFO, &sp)) perror("sched_setscheduler");

    if (mlockall(MCL_CURRENT | MCL_FUTURE)) perror("mlockall");   /* 防止页错误 */

    /* 预先触碰栈和缓冲区，避免运行时首次触碰引发页错误 */
    volatile char stack_warm[64 * 1024];
    memset((void *)stack_warm, 0, sizeof(stack_warm));

    cpu_set_t set; CPU_ZERO(&set); CPU_SET(3, &set);               /* 固定到第 3 核 */
    sched_setaffinity(0, sizeof(set), &set);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    for (;;) {
        add_ns(&next, PERIOD_NS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);  /* 绝对时间睡眠 */

        struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
        /* 计算 latency = now - next，累积统计 */
        /* control_step();  读传感器 -> 计算 -> 写命令（避免 malloc、printf、阻塞 I/O） */
    }
}
```

要点逐条：

1. **SCHED_FIFO**：固定优先级、同优先级先进先出、无时间片；优先级 1–99；调用需要权限（root 或 `CAP_SYS_NICE`）；
2. **`mlockall`**：把当前与将来的页都锁在内存里，禁止换出，消除页错误引起的毫秒级停顿；
3. **预触碰栈**：栈和堆是按需分配的，首次触碰才缺页；预先 memset 一遍；
4. **`clock_nanosleep(..., TIMER_ABSTIME)`**：对绝对时间睡眠，如同 `vTaskDelayUntil`；用相对睡眠（`nanosleep`）会累积漂移；
5. **`CLOCK_MONOTONIC`**：不受系统时间调整的影响；
6. **CPU 亲和与隔离**：把实时线程固定到一个核；内核参数 `isolcpus=3 nohz_full=3 rcu_nocbs=3` 让该核尽量不被其他任务、tick 和 RCU 回调打扰；并把中断亲和 `/proc/irq/N/smp_affinity` 移开该核；
7. **避免**：循环内 `malloc/free`、`printf`、文件 I/O、动态库首次调用、可能阻塞的系统调用、跨线程的普通互斥锁（无继承的 pthread 互斥锁需 `PTHREAD_PRIO_INHERIT` 属性）。

### 8.4 cyclictest

`cyclictest -m -p80 -i2000 -l100000 -a3 -h400` 在 2 ms 周期上测 10 万次从预期唤醒时刻到实际唤醒的**延迟**，输出最小、平均、最大。关键是**最大值**与分布尾部，而不是平均值。要在**负载下**（`stress-ng` 同时压 CPU、内存、IO）测，空载测的结果无意义。常见做法是跑数小时到数十小时。

---

## 9. 时延预算与统计的完整手算

### 9.1 统计示例：平均数的欺骗

某次 cyclictest 取 10 个样本（µs，教学示例）：38, 41, 45, 52, 40, 43, 39, 210, 47, 44。

- 平均：$(38+41+45+52+40+43+39+210+47+44)/10=59.9$ µs；
- 最大：210 µs；
- 排序后第 9 个（约 P90）：52 µs；
- 去掉那一个异常后，其余九个平均约 43.2 µs。

结论：平均 60 µs 看起来不错，但**最大 210 µs 才是实时性的指标**。若控制周期 2 ms，抖动容忍 10%（200 µs），则 210 µs 刚好超出，需要排查异常来源（是否发生了页错误、SMI、热节流、某次大块内存拷贝）。

### 9.2 周期抖动对速度估计的影响

用位置差分估速度：$v=\Delta x/\Delta t$。如果程序假设 $\Delta t=2.00$ ms，实际这一周期为 2.21 ms（抖动 +0.21 ms），则估计速度偏高 $2.21/2.00-1=10.5\%$。所以**用实际时间戳计算 dt**，不要用名义常数（与 P3 文档里"dt 写死 0.001 的名义常数问题"是同一类错误，见 [P3 数字 PID 实现](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/12_数字PID实现.md)）。

### 9.3 端到端时延预算

一条"摄像头 → 视觉 → 控制命令 → 电机"的链路（教学示例）：

| 环节 | 时延（ms） | 备注 |
|---|---|---|
| 相机曝光与传输 | 10 | 帧率 60 Hz 时一帧约 16.7 ms，曝光读出若取半帧约 8–10 |
| V4L2 取帧与预处理 | 3 | 颜色转换、缩放 |
| NPU 推理 | 12 | 小模型 INT8 量级（教学值，非实测） |
| 后处理与 NMS | 2 | |
| 进程间/话题传输 | 1 | 同机 ROS 2 |
| 控制计算 | 1 | |
| CAN 发送与电机响应 | 2 | |
| **合计** | **31** | |

另外在 Linux 上各环节的**抖动**不可忽略：若调度/中断抖动最大 1 ms，总预算里需要预留；若系统没有 RT，最坏抖动可能是 5–10 ms 量级（经验量级，需实测）。P3 的文档给出的视觉端到端延迟约 30–50 ms（本项目取值），与此量级一致（见 [P3 README 自查表](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)）。

```python
lat = [38, 41, 45, 52, 40, 43, 39, 210, 47, 44]
print(sum(lat) / len(lat), max(lat), sorted(lat)[8])         # 59.9 210 52
rest = [x for x in lat if x != 210]
print(sum(rest) / len(rest))                                   # 43.22
print(2.21 / 2.00 - 1)                                          # 0.105
print(10 + 3 + 12 + 2 + 1 + 1 + 2)                              # 31
```

---

## 10. 在七个项目里怎么用

| 项目 | 关联点 | 性质 |
|---|---|---|
| P4 | RK3588 跑 OpenHarmony 标准系统（Linux 内核），摄像头通过 V4L2 设备节点 `/dev/videoN` 取帧（`open/ioctl/mmap/poll`），NPU 通过 RKNN 运行时调用；设备树描述摄像头/NPU 等硬件，由板级 BSP 提供。P4 的工作在用户态应用与模型部署，没有写内核驱动 | 项目事实（V4L2/RKNN）；设备树与驱动为延伸知识 |
| P6 | OpenHarmony 的 HDF 驱动框架与 Linux 内核驱动的关系（统一驱动模型，见 [P6 OpenHarmony 基础](../../01_项目课程/P6_深开鸿M-Robots平台软件/02_OpenHarmony基础_架构_HDF_IPC_构建.md)）；用 C++ 在 OpenHarmony 上做统一设备能力模型 | 项目事实 + 本章概念 |
| P1 / P2 | ROS 2 节点跑在 Linux 上，通过 SocketCAN（`can0` 网络设备）与 CAN 电机通信；周期与抖动受 Linux 调度影响，所以电机侧用安全保护（命令超时）兜底 | 项目事实（SocketCAN）；PREEMPT_RT 为延伸 |
| P5 | 车上 ROS 2 Humble 导航栈在 Linux 运行，控制/定位频率为中低频，抖动要求较低；LiDAR、IMU、里程计的时间戳对齐比硬实时更重要 | 项目事实 |
| P3 / P7 | MCU + RTOS 承担硬实时部分（见 [11 章](./11_实时调度理论_RMS_EDF_优先级反转.md)）；与 PC/视觉的通信走 USB CDC/串口 | 项目事实 |

> **诚实边界**：CV 没有写你配置过 PREEMPT_RT、写过 Linux 内核驱动或设备树。面试时可说："I have worked with Linux as the application platform — V4L2 camera capture, SocketCAN, ROS 2 — and I understand how the device tree and drivers expose hardware, but I haven't written kernel drivers myself." 这样诚实且有深度。

---

## 11. 常见错误与故障现象

1. **设备节点不出现**：设备树 `status` 为 disabled、`compatible` 拼写与驱动不一致、引脚被其他功能复用。
2. **`-EPROBE_DEFER`**：依赖的资源（时钟、GPIO、regulator）未就绪，正常的延迟探测，等待后再试；若一直失败，检查依赖节点。
3. **直接解引用用户指针**：内核崩溃（oops）或安全漏洞；必须 `copy_from_user`。
4. **在中断上下文里睡眠**：`mutex_lock`、`kmalloc(GFP_KERNEL)` 不能在上半部使用；用 `GFP_ATOMIC` 或把工作推迟到线程/工作队列。
5. **实时线程忘记 `mlockall`**：偶发几毫秒停顿（页错误）。
6. **实时线程在循环里 `printf`**：stdout 缓冲与锁、终端写入造成不确定延迟。
7. **只测空载 cyclictest**：负载下延迟差几个数量级。
8. **CPU 频率调节器是 `ondemand`**：时延被降频放大；实时系统设为 `performance`。
9. **RT 线程占满 CPU**：整个系统卡死（除非 RT throttling 起作用）；给 SCHED_FIFO 线程设置合适优先级、不要高于关键内核线程（如中断线程）太多。
10. **用相对睡眠**：`usleep(2000)` 周期 = 2 ms + 执行时间 + 唤醒延迟，持续漂移。
11. **把物理地址当指针用**：Linux 有 MMU，必须 `ioremap`。
12. **忽略时间戳**：用名义 dt 代替实测时间间隔。

---

## 12. 英文面试问答

### Q1. Can you do real-time control on Linux?

**Answer.** It depends on the requirement. A standard Linux kernel is not deterministic: scheduling delay can reach several milliseconds because of long kernel sections and page faults. With the PREEMPT_RT kernel, a pinned CPU core, SCHED_FIFO priority, locked memory, and absolute-time sleeps, we can often get latency in the order of tens to a hundred microseconds, but we must measure the worst case under load with a tool like cyclictest. For a one-kilohertz torque loop or something safety-critical, I prefer a microcontroller with an RTOS. In my projects Linux runs the higher-level nodes, and the real-time inner loops live on the STM32.

**中文译文**：看需求。标准 Linux 内核不是确定性的，长的内核临界区和页错误可能带来数毫秒的调度延迟。用 PREEMPT_RT 内核，固定 CPU 核，SCHED_FIFO 优先级，锁定内存，绝对时间睡眠，通常能把时延降到几十到一百微秒量级，但必须在负载下用 cyclictest 之类的工具测最坏值。对 1 kHz 力矩环或安全关键功能，我更倾向用带 RTOS 的单片机。我的项目里 Linux 跑上层节点，实时内环在 STM32 上。

**Follow-up 1:** What is the first thing you would measure? — The worst-case wake-up latency under load, not the average. （负载下的最坏唤醒延迟，而不是平均值。）

**Follow-up 2:** Have you set up PREEMPT_RT yourself? — No, I have used Linux as an application platform. I understand the mechanisms and I would measure before relying on it. （没有，我把 Linux 当作应用平台；我了解机制，会先测量再依赖。）

### Q2. What is a device tree and why do we need it?

**Answer.** A device tree is a data structure that describes the hardware to the kernel: which peripherals exist, their register addresses, interrupts, clocks and pin settings. Before it, this information was hard-coded in board files in the kernel source. With a device tree, the same kernel binary can boot different boards, because the board-specific description is a separate blob loaded by the bootloader. Drivers match a node by its `compatible` string, and their probe function reads properties from the node. If a device does not appear, I check the status, the compatible string, the pin multiplexing, and the kernel log for probe errors.

**中文译文**：设备树是向内核描述硬件的数据结构：有哪些外设、寄存器地址、中断、时钟和引脚配置。在它之前这些信息硬编码在内核源码的板级文件里。有了设备树，同一个内核二进制可以启动不同的板子，因为板级描述是由 Bootloader 加载的独立数据。驱动通过 `compatible` 字符串匹配节点，probe 函数从节点读取属性。设备不出现时，我检查 status、compatible、引脚复用和内核日志里的 probe 错误。

**Follow-up 1:** What does EPROBE_DEFER mean? — A dependency such as a clock or GPIO driver is not ready, so the kernel retries the probe later. （依赖的时钟或 GPIO 驱动未就绪，内核稍后重试 probe。）

**Follow-up 2:** What is a device tree overlay? — A fragment that modifies the tree, for example to enable an add-on board without rebuilding the whole tree. （修改设备树的片段，如启用扩展板而无需重编整棵树。）

### Q3. Why can't a kernel driver dereference a user pointer directly?

**Answer.** A user pointer is only meaningful in the process's address space. It might be invalid, might point to a page that is swapped out, or might even point into kernel memory if a malicious program passes it. The kernel therefore uses `copy_from_user` and `copy_to_user`, which check the address range and handle page faults safely, and return an error if the access fails. Using a raw pointer can crash the kernel or open a security hole. For fast data paths, we can use `mmap` to share a buffer without copying, which is how V4L2 camera buffers work.

**中文译文**：用户指针只在该进程的地址空间里有意义，可能无效、可能指向已换出的页，恶意程序甚至可能传一个指向内核内存的地址。所以内核用 `copy_from_user` 和 `copy_to_user`，它们检查地址范围并安全处理缺页，失败时返回错误。直接用裸指针可能让内核崩溃或造成安全漏洞。对快速数据通路，可用 `mmap` 共享缓冲而不拷贝，V4L2 摄像头缓冲就是这样工作的。

**Follow-up 1:** What is the cost of a system call? — A user-kernel mode switch, which is small but not free, so we batch work to avoid calling at kilohertz rates without need. （一次用户/内核态切换，开销不大但不是零，所以会合并操作，避免无谓的高频调用。）

**Follow-up 2:** How did you use V4L2? — In the gardening project I captured camera frames through the V4L2 buffer queue with memory-mapped buffers, then preprocessed them for the NPU. （在园艺项目里通过 V4L2 缓冲队列和内存映射缓冲取帧，再为 NPU 预处理。）

### Q4. How would you make a periodic user-space loop on Linux as deterministic as possible?

**Answer.** I would run the thread with SCHED_FIFO at a high priority, pin it to an isolated core, and move interrupts away from that core. I lock memory with mlockall and pre-touch the stack so there are no page faults. I use absolute-time sleep on the monotonic clock, so the period does not drift. I avoid allocation, printing and blocking calls in the loop, and I use priority-inheritance mutexes if I must share data. Then I measure the latency histogram under heavy load, look at the maximum, and compute the time step from real timestamps instead of assuming a fixed period.

**中文译文**：线程用 SCHED_FIFO 高优先级，固定到隔离的 CPU 核，并把中断移出该核。用 mlockall 锁内存并预触碰栈，避免页错误。在单调时钟上用绝对时间睡眠，周期不漂移。循环里避免分配、打印和阻塞调用，必须共享数据时用带优先级继承的互斥量。然后在重负载下测延迟直方图，看最大值，并用真实时间戳计算时间步长，而不是假设固定周期。

**Follow-up 1:** Why absolute time sleep? — Relative sleep adds the execution time and wake-up latency every cycle, so the period drifts. （相对睡眠每周期都累加执行时间和唤醒延迟，周期会漂移。）

**Follow-up 2:** What protects the system if the real-time thread loops forever? — Real-time throttling, which by default limits real-time tasks to ninety-five percent of CPU time. （实时限流，默认限制实时任务最多占 95% CPU 时间。）

---

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 启动链：BootROM → SPL（初始化 DDR）→ U-Boot → 内核 + DTB → init。
2. 设备树把硬件描述从内核代码中分离；`compatible` 匹配驱动，`reg`/`interrupts`/`clocks` 提供资源。
3. 设备不出现：查 `status`、`compatible`、pinctrl、`dmesg` 中的 probe 错误。
4. 用户态必须用 `copy_to_user/copy_from_user`；寄存器访问用 `readl/writel`；物理地址要 `ioremap`。
5. SocketCAN 把 CAN 当网络设备；V4L2 把摄像头当 `/dev/videoN`，用 `mmap` 零拷贝。
6. 上半部要短，下半部可睡眠；PREEMPT_RT 把中断线程化、自旋锁变 rt_mutex。
7. 用户态实时：SCHED_FIFO、`mlockall`、预触碰、`clock_nanosleep(ABSTIME)`、CPU 隔离与亲和。
8. 看最大值不看平均值；在负载下测 cyclictest；dt 用实测时间戳。
9. RT throttling（95%）是安全网。
10. 机器人分层：Linux 管算力与生态，MCU+RTOS 管硬实时内环。

### 自测题

1. 说出嵌入式 Linux 从上电到用户空间经过的几个阶段，以及为什么 DDR 初始化放在 SPL。
2. 设备树节点里 `compatible = "a,b", "a,generic"` 表示什么？
3. 为什么内核里不能用普通指针访问用户缓冲？
4. 为什么实时线程需要 `mlockall`？不预触碰栈会怎样？
5. cyclictest 平均 30 µs，最大 3 ms，这个系统适合 2 ms 周期的控制环吗？

**参考答案要点**：1. BootROM、SPL、（安全固件）、U-Boot、内核、init；BootROM 阶段只有片内 SRAM，需要先初始化 DDR 才能装载大程序。2. 从具体到通用的匹配列表，先匹配具体驱动，没有再匹配通用驱动。3. 用户指针可能无效/换出/越权，必须经 `copy_*_user` 检查与处理缺页。4. 避免页错误引起的毫秒级阻塞；不预触碰则首次访问时缺页。5. 不适合：最大值 3 ms 超过周期，必须找出原因（页错误、频率调节、其他中断）或把内环放到 MCU。

### 相关链接

- 上一章：[实时调度理论：RMS、EDF 与优先级反转](./11_实时调度理论_RMS_EDF_优先级反转.md)
- 相关：[OpenHarmony、Hi3861、RK3588 平台基础](./06_OpenHarmony_Hi3861_RK3588平台基础.md)、[UART、SPI、I2C、以太网、TCP](./04_UART_SPI_I2C_以太网_TCP.md)、[CAN 与 FDCAN](./03_CAN_FDCAN与电机通信.md)、[Linux 与 Shell、Git](../12_机器人软件/01_Linux与Shell_Git.md)、[DDS 与分布式通信](../12_机器人软件/05_DDS与分布式通信.md)
- 八股：[操作系统](../../03_八股知识/01_计算机基础/02_操作系统.md)、[Linux、Git、编译构建八股](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)
- 项目：[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)、[P2](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)
