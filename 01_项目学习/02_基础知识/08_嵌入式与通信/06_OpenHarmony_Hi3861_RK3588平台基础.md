# OpenHarmony、Hi3861 与 RK3588 平台基础

> **一句话定位**：P4（智能园艺）和 P6（M-Robots 平台）都跑在 OpenHarmony 生态上，但硬件天差地别：Hi3861 是只有几百 KB 内存的 Wi-Fi 微控制器（LiteOS-M 内核），RK3588 是八核 ARM + NPU 的"小电脑"（Linux 内核）。这一章把操作系统分层、两类内核的差别、外设（GPIO/ADC/PWM/Wi-Fi、摄像头 V4L2、NPU/VPU）、交叉编译与部署流程讲清楚，让你能回答"这套东西是怎么搭起来、怎么跑起来的"。

**本章学完你能：**

- 讲清 OpenHarmony 的分层架构与"轻量/小型/标准"三类系统；
- 对比 LiteOS-M 与 Linux 内核：有无 MMU、进程模型、调度、内存、驱动方式；
- 说出 Hi3861 的资源量级与 Wi-Fi/GPIO/ADC/PWM 的使用思路，并手算一个"土壤湿度 → 水泵状态机（带迟滞）"；
- 说出 RK3588 的 CPU/NPU/VPU/ISP 分工，走通 V4L2 取图流程，并手算图像带宽、预处理缩放与 NPU 理论耗时；
- 解释交叉编译（host/target、toolchain、sysroot）与部署（烧录、hdc/ssh/scp）；
- 用英文回答"为什么一个系统里同时用 MCU 和 SoC"。

**前置知识：**[STM32 章](./01_STM32_时钟_GPIO_中断_DMA.md)（时钟、GPIO、DMA）；[FreeRTOS 章](./02_FreeRTOS实时系统.md)（RTOS 概念）；[Linux 与 Shell、Git](../09_机器人软件/01_Linux与Shell_Git.md)。

**相关项目：**[P4 OpenHarmony 智能园艺系统](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)（RK3588 边缘视觉 + Hi3861 灌溉）；[P6 深开鸿 M-Robots 平台软件](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)（OpenHarmony 上的 C++ 设备抽象）。

## 目录

1. OpenHarmony 架构分层
2. 两类内核：LiteOS-M 与 Linux
3. Hi3861 平台
4. 例：土壤湿度到水泵的迟滞状态机（手算）
5. RK3588 平台
6. 摄像头与 V4L2；图像带宽与预处理手算
7. NPU 推理的理论耗时估算
8. 交叉编译
9. 部署与调试
10. 为什么 MCU + SoC 组合
11. 常见错误与故障现象
12. 面试问答
13. 要点速记、自测题、链接

---

## 1. OpenHarmony 架构分层

### 1.1 它要解决什么问题

智能设备种类极多：手环（几十 KB 内存）、摄像头、手机、电视、机器人。传统做法是每类设备一个操作系统。OpenHarmony 的目标是**一套分层、可裁剪的系统，在不同资源量级的设备上跑，并让设备之间能互联协同**。

### 1.2 分层（自底向上）

```text
应用层      应用 / 服务（ArkTS 应用、C/C++ 原生服务）
框架层      应用框架、ArkUI、ability 管理等
系统服务层  分布式软总线、分布式数据管理、设备管理、基础服务
内核层      LiteOS-M / LiteOS-A / Linux  +  驱动框架 HDF
硬件        MCU / SoC
```

- **内核层**：按设备能力可选内核（见下节）；
- **驱动框架 HDF（Hardware Driver Foundation）**：提供统一的驱动模型与硬件抽象，让同一份驱动在不同内核上复用；
- **系统服务层**：其中**分布式软总线**让设备自动发现、连接、传输数据——是多设备协同的基础；P6 的 M-Robots 平台"设备发现、能力注册、状态同步、任务分发"在概念上与此同类，但 P6 在应用/平台层基于 M-DDS 实现，具体实现以 P6 项目文档为准。

### 1.3 三类系统（按资源量级）

| 系统类型 | 典型内存 | 内核 | 典型设备 |
|---|---|---|---|
| 轻量系统（mini system） | 128 KB 起 | LiteOS-M | MCU 类：Hi3861、传感器、手环 |
| 小型系统（small system） | 1 MB 起 | LiteOS-A（或 Linux） | 摄像头、行车记录仪等带 MMU 的设备 |
| 标准系统（standard system） | 128 MB 起 | Linux（或 LiteOS-A） | 手机、平板、RK3588 开发板 |

（量级为官方文档常见描述，具体数值以所用版本文档为准。）

### 1.4 在项目里怎么用

P4：RK3588 跑**标准系统**（Linux 内核）承担摄像头 + NPU 推理 + 网络；Hi3861 跑**轻量系统**（LiteOS-M）控制水泵。两者都在 OpenHarmony 生态里，但开发方式完全不同。P6：在 OpenHarmony 上用 C++ 做统一设备能力模型，把 ROS/ROS2 运动指令映射到底层驱动。

---

## 2. 两类内核：LiteOS-M 与 Linux

### 2.1 核心差别

| 维度 | LiteOS-M（Hi3861） | Linux（RK3588） |
|---|---|---|
| 硬件要求 | 无 MMU，内存 KB~MB | 有 MMU，内存 GB |
| 地址空间 | **单一物理地址空间**，所有任务共享 | 每进程独立虚拟地址空间 |
| 执行单元 | 任务（线程），无进程隔离 | 进程 + 线程 |
| 调度 | 抢占式、优先级 + 同级时间片（与 FreeRTOS 类似） | CFS 等，也有实时类（SCHED_FIFO/RR） |
| 内存管理 | 静态/固定内存池、简单堆 | 虚拟内存、分页、swap、`malloc` 大块 |
| 隔离 | 任务写坏内存会拖垮整机 | 进程崩溃不影响内核与其他进程 |
| 驱动 | 直接访问寄存器/HAL，运行在同一空间 | 内核模块 + 设备节点 `/dev/xxx` + 用户态访问 |
| 启动时间 | 毫秒级 | 秒级 |
| 实时性 | 好（确定的中断延迟与调度） | 一般，需 PREEMPT_RT 等增强 |
| API | CMSIS-RTOS2（`osThreadNew`、`osDelay`、`osMutexNew`……）、POSIX 子集 | 完整 POSIX/Linux |

LiteOS-M 的编程感觉与 [FreeRTOS](./02_FreeRTOS实时系统.md) 非常接近：创建任务、延时、互斥量、消息队列；差别主要在 API 前缀与板级支持包。

### 2.2 为什么这样分工

- **MCU（LiteOS-M）**：做确定性、低功耗、低成本的"最后一米"——读传感器、驱动继电器、保持本地闭环；掉电重启快；
- **SoC（Linux）**：做算力密集和网络密集的事——视觉推理、数据库、Web 服务、Agent；
- 两者通过 Wi-Fi/串口/网络协议连接，各司其职，MCU 即便 SoC 重启也能保持灌溉的安全状态（见第 4 节"本地闭环"）。

---

## 3. Hi3861 平台

### 3.1 资源量级（典型规格，以数据手册和所用 SDK 为准）

Hi3861 是一颗面向 IoT 的 Wi-Fi SoC：32 位 RISC-V 内核，主频约 160 MHz，SRAM 约 352 KB，ROM 约 288 KB，片外 Flash 约 2 MB，支持 2.4 GHz Wi-Fi（802.11 b/g/n）。资源很紧：**每个任务栈几 KB，整个堆有限**，不能随意 `malloc` 大块。

**手算：内存预算（教学示例）**：352 KB SRAM 里，系统与 Wi-Fi 协议栈先占用一大部分，假设留给应用约 100 KB。应用要 4 个任务，每个栈 4 KB = 16 KB；一个 TCP 收发缓冲 2 × 2 KB = 4 KB；传感器缓冲 1 KB；共 21 KB，剩 79 KB 余量。经验：嵌入式应用要**静态规划**内存，运行中不在循环里反复 `malloc/free`（碎片化）。

### 3.2 开发与构建

- 工具：`hb`（OpenHarmony 的构建命令行）：`hb set` 选产品，`hb build` 编译；底层是 **GN + Ninja**，每个模块有 `BUILD.gn`；
- 输出：一个固件 `.bin`，通过串口烧录工具（如 HiBurn）写入 Flash；
- 应用入口：用 `SYS_RUN(MyApp)` 或类似宏注册入口函数，在入口里创建任务：

```c
// 教学示例，非项目源码；API 名称以所用 SDK 头文件为准
#include "cmsis_os2.h"
static void MainTask(void *arg)
{
    for (;;) {
        // 读传感器，更新状态机，驱动水泵
        osDelay(100);          // 以 tick 为单位，具体 tick 率看配置
    }
}
static void AppEntry(void)
{
    osThreadAttr_t attr = { .name = "main", .stack_size = 4096, .priority = osPriorityNormal };
    osThreadNew(MainTask, NULL, &attr);
}
```

### 3.3 外设

| 外设 | 概念 | 使用要点 |
|---|---|---|
| GPIO | 数字输入输出 | 初始化引脚复用 → 设方向 → 读/写电平（常见 IoT HAL：`IoTGpioInit`、`IoTGpioSetDir`、`IoTGpioSetOutputVal`） |
| ADC | 模数转换，读模拟传感器（土壤湿度、光敏） | 分辨率 N 位，读数 0 到 $2^N-1$；注意**参考电压与输入范围**，超量程会饱和 |
| PWM | 脉宽调制，调速/调光/舵机 | 设置频率与占空比（`IoTPwmStart(port, duty, freq)` 之类） |
| UART / I2C / SPI | 同 [前几章](./04_UART_SPI_I2C_以太网_TCP.md) | 与其它模块通信 |
| Wi-Fi | STA（连路由器）/ AP（做热点） | 连接 → DHCP → socket 通信 |

### 3.4 Wi-Fi 联网

典型流程：初始化 Wi-Fi → 扫描 → 以 STA 模式连接指定 SSID/密码 → 等待 DHCP 获得 IP → 创建 socket（TCP/UDP）或 MQTT 客户端连接服务器。实际项目里要加**断线重连**与**超时**，因为 Wi-Fi 随时可能掉线；**本地控制逻辑不能依赖网络**——即使断网，水泵状态机仍按本地阈值工作。

---

## 4. 例：土壤湿度到水泵的迟滞状态机（手算）

P4 的 CV 描述：*"implemented a soil-moisture-driven pump state machine on Hi3861 with independent start/stop thresholds and manual-priority override for a stable local closed loop."* 下面用教学数值说明这类设计背后的道理（**数值均为示例，非项目参数**）。

### 4.1 ADC 到湿度百分比

电容式土壤湿度传感器：越干读数越大（假设）。标定两点：空气中（全干）$raw_{dry}=3200$，水中（全湿）$raw_{wet}=1200$（12 位 ADC，0–4095）。线性映射：

$$m(\%)=\frac{raw_{dry}-raw}{raw_{dry}-raw_{wet}}\times100$$

$raw=2200$：$m=(3200-2200)/(3200-1200)\times100=50\%$；$raw=2800$：$m=20\%$。取值要夹到 0–100%，并对多次采样**取平均/中值滤波**（例如 8 次中值）消除单次噪声。

### 4.2 为什么需要迟滞（两个独立阈值）

只用一个阈值 30%（<30% 开泵，≥30% 关泵）：湿度在 30% 附近有 ±3% 的噪声时，会在 29%、31%、29%… 之间来回，水泵疯狂开关（**抖振**），损坏继电器与水泵。

引入**迟滞**：开泵阈值 $T_{on}=30\%$，关泵阈值 $T_{off}=45\%$（$T_{off}>T_{on}$）。

- 状态 OFF：当 $m<T_{on}$ 才转为 ON；
- 状态 ON：当 $m>T_{off}$ 才转为 OFF；
- 中间 30%–45%：保持原状态。

噪声 ±3% 不可能同时跨越两个相距 15% 的阈值，抖振消除。

### 4.3 手算时间线

| 时刻 | 湿度 m | 当前状态 | 判断 | 新状态 |
|---|---|---|---|---|
| t0 | 35% | OFF | 35 ≥ 30 | OFF |
| t1 | 29% | OFF | 29 < 30 | **ON** |
| t2 | 33% | ON | 33 ≤ 45 | ON |
| t3 | 44% | ON | 44 ≤ 45 | ON |
| t4 | 46% | ON | 46 > 45 | **OFF** |
| t5 | 40% | OFF | 40 ≥ 30 | OFF |

### 4.4 手动优先与安全限制

- **手动优先（override）**：用户通过网页/按钮手动开/关泵时，手动指令优先于自动逻辑，直到手动模式结束或超时；自动逻辑只在无手动指令时生效；
- **最长运行时间**：防止传感器故障（一直读到"干"）导致水泵无限运行，设上限（如连续运行不超过 T_max，之后强制关闭并报警）；
- **最短切换间隔**：保护水泵；
- **传感器合法性**：读数超出标定范围很多（断线读到 0 或 4095）→ 标记故障，进入安全状态（关泵）；
- **来自上层（Agent/服务器）的请求**：作为"建议/受约束的请求"，在本地做限幅和合法性检查后才执行，不能直接绕过本地安全逻辑。

这与 P6 的"action-proposal → policy-check → execution → state-feedback"思想一脉相承：上层只提议，下层有最终的安全检查。

---

## 5. RK3588 平台

### 5.1 模块分工（典型规格，以芯片手册为准）

| 模块 | 典型配置 | 用途 |
|---|---|---|
| CPU | 4×Cortex-A76 + 4×Cortex-A55（big.LITTLE） | 通用计算、业务逻辑、图像前后处理 |
| GPU | Mali-G610 | 图形/部分通用计算 |
| NPU | 约 6 TOPS（INT8 峰值，3 个核心） | 神经网络推理（RKNN） |
| VPU | 硬件视频编解码（支持 8K 级解码，多路 1080p） | 视频流编解码，减轻 CPU |
| ISP | 图像信号处理器 | 摄像头 RAW → YUV，自动曝光/白平衡 |
| 接口 | MIPI CSI（摄像头）、USB 3、PCIe、千兆以太网、HDMI | |

要点：**不同类型的活要交给对应的硬件单元**——推理给 NPU，视频编解码给 VPU，图像处理给 ISP/RGA（2D 图形加速），CPU 负责调度与逻辑。这样 CPU 才不会成为瓶颈。

### 5.2 启动与系统

RK3588 板子通常有 Bootloader（U-Boot）→ 内核（Linux，OpenHarmony 标准系统用的内核）→ 用户空间。设备树（Device Tree）描述硬件（哪些外设、地址、中断）；驱动按设备树匹配。

---

## 6. 摄像头与 V4L2；图像带宽与预处理手算

### 6.1 V4L2 是什么

**V4L2（Video4Linux2）** 是 Linux 的视频设备驱动接口。摄像头在系统里是 `/dev/video0` 这样的设备节点，用户程序通过 `ioctl` 与之交互。

### 6.2 取图标准流程（mmap 流式方式）

```text
1. open("/dev/video0")
2. VIDIOC_QUERYCAP        查询能力（是否支持视频捕获、流式 I/O）
3. VIDIOC_S_FMT           设置分辨率与像素格式（如 1920x1080, NV12 或 MJPEG）
4. VIDIOC_REQBUFS         向驱动申请 N 个缓冲区（常用 3-4 个）
5. 对每个缓冲区 VIDIOC_QUERYBUF + mmap 映射到用户空间
6. VIDIOC_QBUF            把缓冲区全部"入队"，交给驱动填充
7. VIDIOC_STREAMON        开始采集
8. 循环： poll/select 等待 -> VIDIOC_DQBUF 取出一个已填满的缓冲区
          -> 处理该帧（拷贝或直接送 NPU/编码器）
          -> VIDIOC_QBUF 把缓冲区还给驱动
9. VIDIOC_STREAMOFF，munmap，close
```

要点：**缓冲区循环使用**（入队—出队—处理—再入队），处理时间太长会让驱动没有空闲缓冲区可填而**丢帧**。多缓冲（3–4 个）提供抖动余量。mmap 避免内核到用户空间的拷贝；更进一步用 DMA-BUF 在摄像头、NPU、编码器之间零拷贝传递。

### 6.3 手算：帧大小与带宽

- 1920×1080 的 NV12（YUV420，每像素 1.5 字节）：$1920\times1080\times1.5=3{,}110{,}400$ B ≈ 3.11 MB；
- 30 fps 带宽：$3.11\times30=93.3$ MB/s；
- 如果是 RGB24：$1920\times1080\times3=6.22$ MB/帧，30 fps = 187 MB/s，是 NV12 的 2 倍；
- 若处理一帧 CPU 拷贝耗时过长，就会丢帧；所以倾向用硬件加速（RGA/DMA-BUF）做缩放与格式转换。

### 6.4 手算：为 YOLO 做预处理（letterbox）

YOLOv5 输入 640×640。原图 1920×1080，保持纵横比缩放：

1. 缩放比 $s=\min(640/1920,\ 640/1080)=\min(0.3333,\ 0.5926)=0.3333$；
2. 新尺寸 $=640\times360$（$1080\times0.3333=360$）；
3. 垂直方向填充 $640-360=280$ 行，上下各 140 行（常填灰色 114）；
4. 输入张量：$640\times640\times3=1{,}228{,}800$ B ≈ 1.23 MB（INT8/UINT8 输入）；
5. 推理后，检测框坐标要反变换：$x_{orig}=(x_{net}-pad_x)/s$，$y_{orig}=(y_{net}-pad_y)/s=(y_{net}-140)/0.3333$。

忘记 letterbox 反变换，框会整体偏位并纵向压缩——这是"检测框位置不对"的常见原因。

---

## 7. NPU 推理的理论耗时估算

### 7.1 量级估算

YOLOv5s 在 640×640 下约 7.2 M 参数、约 16.5 GFLOPs（公开常见数字，量级参考）。NPU INT8 峰值 6 TOPS：

$$t_{theory}=\frac{16.5\times10^9}{6\times10^{12}}=2.75\ \text{ms}$$

**这是 100% 利用率的下限**，实际硬件利用率（算子是否支持、内存带宽、层间等待）通常远低于峰值，一个粗略的预期是峰值的 10%–30%，对应 **约 10–30 ms/帧**（推算，仅为量级）。再加上前处理（缩放、格式转换）、后处理（NMS）和数据搬运，端到端可能是几十毫秒。

**提醒**：项目里真实的帧率、延迟必须来自实测。【待你补全：P4 中 RK3588 上 YOLOv5 INT8 的实测推理耗时/帧率、输入分辨率、置信度阈值；若没有精确数字，面试时可说"我没有记住确切数字，大致是…，并说明怎么测"】。

### 7.2 INT8 量化与部署链

PC 端用 **RKNN-Toolkit2** 将训练好的模型（ONNX 等）转换为 `.rknn`，量化时用一批代表性校准图片统计激活范围；板端用 RKNN Runtime（C API：`rknn_init`、设置输入、`rknn_run`、`rknn_outputs_get`）加载 `.rknn` 推理。量化原理与精度损失见 [模型量化与边缘部署 RKNN](../07_感知与机器学习/04_模型量化与边缘部署_RKNN.md)，网络本身见 [卷积网络与目标检测 YOLO](../07_感知与机器学习/03_卷积网络与目标检测_YOLO.md)。

---

## 8. 交叉编译

### 8.1 要解决什么问题

RK3588/Hi3861 算力和工具链都不适合在板子上编译大型工程；我们在 x86 PC（**host**）上编译出运行在 ARM/RISC-V 板子（**target**）上的程序，叫**交叉编译**。

### 8.2 关键概念

- **工具链（toolchain）**：交叉编译器，前缀表示目标，如 `aarch64-linux-gnu-gcc`（64 位 ARM Linux）、`arm-none-eabi-gcc`（裸机 Cortex-M）、`riscv32-...-gcc`；
- **sysroot**：目标板的头文件与库（libc、依赖库）的拷贝，链接时用 `--sysroot` 指向；
- **架构不一致的后果**：把 x86 编出来的二进制拷到板子上会报 "Exec format error"。用 `file a.out` 查看：`ELF 64-bit LSB executable, ARM aarch64` 才是对的。

### 8.3 CMake 工具链文件（教学示例）

```cmake
# toolchain-aarch64.cmake
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /opt/sysroot-aarch64)          # 目标板的 sysroot
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)     # 程序从 host 找
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)      # 库只从 sysroot 找
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

使用：`cmake -B build -DCMAKE_TOOLCHAIN_FILE=toolchain-aarch64.cmake && cmake --build build`。更多 CMake 用法见 [C++ 现代特性与工程实践](../09_机器人软件/02_C++现代特性与工程实践.md)。

### 8.4 常见坑

- 库架构不匹配（链接了 x86 的 `.so`）；
- glibc 版本比目标板新 → 运行时找不到符号版本（`GLIBC_2.xx not found`），解决：用与板子匹配的 sysroot 或静态链接；
- 浮点 ABI（软浮点/硬浮点）不一致；
- OpenHarmony 用自己的构建系统（GN + Ninja + 自带工具链），不是随便用系统 gcc。

---

## 9. 部署与调试

### 9.1 Hi3861（轻量系统）

1. `hb build` 得固件；
2. 通过 USB 转串口连接板子，用烧录工具把 `.bin` 写入 Flash（需要复位/按键进入烧录模式）；
3. 串口终端查看日志（`printf`/`HILOG`），这是主要调试手段；
4. Wi-Fi 联网后可用服务器端日志验证。

### 9.2 RK3588（标准系统/Linux）

1. 交叉编译得到可执行文件和依赖库；
2. 用 `scp`/`rsync`（SSH）或 OpenHarmony 的 `hdc`（设备连接工具）`hdc file send` 推到板子；
3. `hdc shell` 或 `ssh` 登录运行；
4. 日志：`hilog`（OpenHarmony）或 `journalctl`/`dmesg`（Linux）；
5. 服务化：用 init 配置（OpenHarmony 的 `.cfg`）或 systemd 单元（普通 Linux）让程序随系统启动，崩溃自动重启（见 [Linux 与 Shell、Git 的 systemd 一节](../09_机器人软件/01_Linux与Shell_Git.md)）。

### 9.3 排查顺序（通用）

物理连接/供电 → 串口日志（启动是否成功）→ 驱动是否加载（`ls /dev/video*`）→ 权限 → 依赖库 → 业务日志 → 抓包/示波器。

---

## 10. 为什么 MCU + SoC 组合

> 面试官常问："为什么灌溉控制放在 Hi3861 上，而不是全放在 RK3588 上？"

要点：

1. **实时与可靠**：MCU 上电秒级启动，不会因为 Linux 卡顿/重启影响水泵控制；本地状态机即使网络/SoC 离线也能维持安全行为；
2. **成本与功耗**：MCU 几元、毫瓦级；SoC 瓦级；
3. **隔离风险**：高层（视觉、Agent）可能出错、更新频繁；执行层保持简单、确定、可审计；
4. **分层职责**：SoC 给出观察与建议，MCU 做受约束的执行；这与 P6 "action-proposal → policy-check → execution → state-feedback" 的安全执行链同构。

---

## 11. 常见错误 / 边界 / 故障现象

| 现象 | 常见原因 |
|---|---|
| Hi3861 烧录后无输出 | 烧录模式/波特率错；入口没注册；栈太小崩溃 |
| 任务莫名重启 | 栈溢出；看门狗；堆耗尽 |
| ADC 读数飘/跳 | 参考电压；供电噪声；未做滤波；输入阻抗不匹配 |
| 水泵频繁开关 | 单阈值无迟滞；采样噪声 |
| 本地逻辑依赖网络 | 断网即失控；应本地兜底 |
| RK3588 摄像头无画面 | 设备节点没出现；格式/分辨率不支持；权限；设备被占用 |
| 取图丢帧 | 处理时间 > 帧间隔；缓冲区太少；CPU 拷贝过多 |
| 检测框偏位 | letterbox 反变换遗漏；BGR/RGB 顺序；归一化不一致 |
| NPU 精度下降明显 | 量化校准集不具代表性；算子被量化破坏 |
| 板子上 "Exec format error" | 架构不匹配 |
| 找不到 `libxxx.so` | 依赖库没部署；`LD_LIBRARY_PATH` |

---

## 12. 面试怎么问、怎么答

**Q1. What is OpenHarmony and how did you use it?**

> OpenHarmony is an open-source, layered operating system for devices of very different sizes. Its kernel layer can be LiteOS-M for microcontrollers or Linux for richer devices, and a driver framework and system services sit above that. In my gardening project I used a Hi3861 microcontroller on the lightweight system for pump control and an RK3588 board on the standard system for camera inference. In my current internship, I write C++ components on OpenHarmony for a robot platform.

追问：LiteOS-M 与 Linux 的区别？（见 2.1 表）；HDF 是什么？（统一驱动模型）。

**Q2. Why did you use a hysteresis in the pump controller?**

> A single threshold causes chattering when the sensor noise crosses it: the pump would switch on and off rapidly. With separate start and stop thresholds, the state only changes when the moisture crosses the far threshold, so noise within the band doesn't trigger switching. I also gave manual commands priority over the automatic logic and kept the logic local, so it works even if the network or the vision side is down.

【待你补全：P4 的实际阈值与最短/最长运行限制，采样滤波方式；若不确定用"example thresholds"说明】

**Q3. How do you run a YOLO model on RK3588?**

> I train and export the model on a PC, convert it with RKNN-Toolkit to an INT8 model using a calibration set, and run it on the board with the RKNN runtime on the NPU. The pipeline is camera frame, resize with letterbox, inference, decode boxes, non-maximum suppression, then map boxes back to the original image. I attach a timestamp and confidence to each observation so the decision layer can judge how fresh and reliable it is.

追问：INT8 为什么快？（权重/激活 8 位，带宽小、MAC 并行度高）；精度掉了怎么办？（校准集、混合精度、量化感知训练）。

**Q4. What is cross-compilation?**

> I build on an x86 host with a toolchain that targets the board's architecture, for example aarch64 for RK3588, and link against a sysroot that matches the board's libraries. I check the result with the `file` command and make sure the C library versions are compatible.

---

## 本章要点速记

1. OpenHarmony：内核层（LiteOS-M / LiteOS-A / Linux + HDF）→ 系统服务 → 框架 → 应用；按资源分轻量 / 小型 / 标准系统；
2. LiteOS-M：无 MMU、单一地址空间、类 RTOS；Linux：MMU、进程隔离、完整 POSIX；
3. Hi3861：几百 KB RAM 级 Wi-Fi MCU，`hb build`，GPIO/ADC/PWM/Wi-Fi；内存要静态规划；
4. 迟滞控制：$T_{on}<T_{off}$ 消除抖振，加手动优先、最长运行时间、传感器合法性检查、本地兜底；
5. RK3588：A76 + A55、NPU（约 6 TOPS INT8）、VPU、ISP；各负其责；
6. V4L2 流程：open → S_FMT → REQBUFS → mmap → QBUF → STREAMON → DQBUF/处理/QBUF 循环；
7. 1080p NV12 = 3.11 MB/帧，30 fps = 93 MB/s；letterbox 先算 $s$ 再算填充，输出框要反变换；
8. NPU 理论耗时 = 运算量/峰值，实际按 10–30% 利用率估，真实数据必须实测；
9. 交叉编译：toolchain + sysroot，用 `file` 验证架构；
10. MCU + SoC：执行层简单确定、本地兜底，SoC 负责感知与决策。

## 自测题

1. 一个 352 KB RAM、无 MMU 的 Wi-Fi 传感器节点，和一个 8 GB RAM 的视觉网关，各适合哪类系统与内核？为什么前者不能跑 Linux？（要点：前者轻量系统 / LiteOS-M；后者标准系统 / Linux；Linux 需要 MMU 与较大内存做虚拟内存和进程隔离。）
2. 湿度 $T_{on}=25\%$、$T_{off}=40\%$，序列 30, 24, 28, 38, 41, 35, 22，起始 OFF，写每步状态。（要点：30 OFF；24 → ON；28 ON；38 ON；41 → OFF；35 OFF；22 → ON。）
3. 标定 $raw_{dry}=3000$、$raw_{wet}=1000$，raw = 1800 的湿度？（要点：(3000−1800)/2000 = 60%。）
4. 1280×720 NV12，25 fps 的带宽？（要点：1280×720×1.5 = 1.382 MB；×25 = 34.6 MB/s。）
5. 1280×720 图 letterbox 到 640×640，缩放比、填充？（要点：s = min(0.5, 0.889) = 0.5，新尺寸 640×360，上下各填 140。）
6. 为什么 x86 编的程序拷到 RK3588 跑不起来？怎么确认？（要点：指令集不同；`file` 查看 ELF 架构。）

## 相关链接

- 上一章：[电机、编码器、驱动与力矩控制](./05_电机_编码器_驱动与力矩控制.md)
- 相关：[模型量化与边缘部署 RKNN](../07_感知与机器学习/04_模型量化与边缘部署_RKNN.md)、[Transformer、LLM 与 Agent、LangGraph](../07_感知与机器学习/05_Transformer_LLM与Agent_LangGraph.md)、[DDS 与分布式通信](../09_机器人软件/05_DDS与分布式通信.md)
- 八股：[Linux、Git、编译构建八股](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)
- 项目：[P4](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
