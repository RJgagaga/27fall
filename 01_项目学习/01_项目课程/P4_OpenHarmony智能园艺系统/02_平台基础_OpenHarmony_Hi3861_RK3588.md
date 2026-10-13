# 02 平台基础：OpenHarmony、Hi3861、RK3588

> **一句话定位**：OpenHarmony 是操作系统家族，Hi3861 是跑它轻量版本的 Wi-Fi MCU，RK3588 是带 NPU 的应用级 SoC；P4 把它们按"算力、实时性、功耗、成本"分工，而不是随便选了三块板子。

## 本章学完你能……

- 说清 OpenHarmony 的**系统类型**（轻量/小型/标准）与**内核**（LiteOS-M / LiteOS-A / Linux）的对应关系，以及 P4 里它出现在哪。
- 说出 Hi3861 的定位、资源量级、外设、开发流程，知道它**能做什么、不能做什么**。
- 说出 RK3588 的定位、NPU 与 RKNN 工具链（RKNN-Toolkit2 / RKNPU2 运行库）的分工。
- 用**一道手算**证明"YOLO 不可能放在 Hi3861 上"，同时估算 NPU 理论下界。
- 回答"为什么这样分工"的面试题。

## 前置知识

- 通用基础：[OpenHarmony、Hi3861、RK3588 平台基础（基础篇）](../../02_基础知识/08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)（本章是它的项目化版本）
- [STM32 时钟、GPIO、中断、DMA](../../02_基础知识/08_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)（MCU 通识）
- 上一章：[01 系统总览与三层架构](./01_系统总览与三层架构.md)

## 目录

1. 为什么需要这一章
2. OpenHarmony 是什么
3. Hi3861
4. RK3588
5. 为什么这样分工：一组手算
6. 启动与部署流程
7. 常见故障与排查
8. 面试怎么问、怎么答
9. 要点速记与自测

> **关于数字的说明**：本章的芯片规格来自厂商/社区的**公开资料**，面试时说"according to the public datasheet"；本项目的板子型号、内存、系统镜像、实测耗时采用教材统一取值（见 README 自查表），与实机不一致时以实机为准并同步修改。手算部分均用 Python 复核过（见 5.6 节）。

---

## 1. 为什么需要这一章

CV 里有三个平台名词：OpenHarmony、RK3588、（隐含的）Hi3861。面试官（尤其是 AI/机器人方向）常常**不熟悉 OpenHarmony**，会问：

- "What is OpenHarmony? Is it like Android?"
- "Why use a microcontroller for irrigation and a separate board for vision?"
- "What can the NPU do that the CPU can't?"

答得好的关键，是把名词变成**能力边界**：这块板子有多少内存、能不能跑某个模型、断网时谁负责。

---

## 2. OpenHarmony 是什么

### 2.1 定位

OpenHarmony（开放原子开源基金会孵化的开源操作系统项目）是面向**全场景智能设备**的分布式操作系统；华为的 HarmonyOS 是基于它的商业发行版之一（说法：OpenHarmony 是开源底座，HarmonyOS 是商业发行）。它不是"Android 的改版"，内核、框架与应用模型都是自己的体系，但在手机端有相似的"应用 + 框架 + 内核"分层。

### 2.2 三种系统类型与内核

OpenHarmony 按设备资源分为三类系统（公开文档的说法）：

| 系统类型 | 典型设备 | 内存量级 | 常用内核 | 本项目对应 |
|---|---|---|---|---|
| 轻量系统（mini system） | MCU 类：传感器节点、智能开关 | 128 KB～几 MB | **LiteOS-M** | **Hi3861** |
| 小型系统（small system） | 带 MMU 的小设备：摄像头、智能家居面板 | 1 MB～几百 MB | LiteOS-A（或 Linux） | — |
| 标准系统（standard system） | 手机、平板、开发板 | ≥128 MB | Linux（或 LiteOS-A） | RK3588 若运行 OpenHarmony 标准系统则属此类 |

> **注意**：RK3588 板端运行的是 **Linux 用户态程序**（V4L2 取帧、RKNN 运行库推理），而不是 OpenHarmony 标准系统镜像。本项目取值：Orange Pi 5 Plus（RK3588，8 GB），Ubuntu 22.04 Rockchip 镜像，内核 5.10，RKNN-Toolkit2 1.6.0 与 librknnrt 1.6.0。V4L2 和 RKNN 都属于 Linux 内核/驱动层的接口，面试中说 "the RK3588 runs the vision process in userspace on top of the Linux kernel interfaces (V4L2 and the RKNN runtime)" 是准确的说法。因此 P4 里 OpenHarmony 只出现在 Hi3861 节点上，不要说成"整个系统跑在 OpenHarmony 上"。

### 2.3 关键技术点（面试能说到这个深度就够）

- **内核抽象层（KAL）/ CMSIS 接口**：让同一份应用代码在不同内核上编译；轻量系统常用 CMSIS-RTOS2 风格的任务、互斥锁、消息队列 API。
- **HDF（Hardware Driver Foundation）**：驱动框架，把外设驱动做成"可配置、可复用"的模块；小型/标准系统用得较多。
- **分布式软总线（DSoftBus）与设备虚拟化**：让设备之间发现、认证、传输数据；是"全场景协同"的底座。（P4 没有用软总线，设备间是普通 Wi-Fi + TCP，不要在面试里夸大。）
- **应用开发**：轻量系统用 C/C++（本项目的 Hi3861 部分）；标准系统用 ArkTS/ArkUI。

### 2.4 在 P4 里的位置

- **Hi3861 的固件**：基于 OpenHarmony 轻量系统（LiteOS-M），任务、定时、GPIO/ADC 驱动接口、Wi-Fi、lwIP socket 都来自 SDK。
- **竞赛背景**：CV 写 "9th Huawei ICT Competition, Programming (OpenHarmony) — Third Prize, China National Final; First Prize, Provincial Round"，与 P4 相关。回答时只说 CV 有的事实，不要补充排名人数、评委评价等没依据的细节。

### 2.5 三类系统与内核的差异（可展开的深度）

| 维度 | 轻量系统（LiteOS-M） | 小型系统（LiteOS-A） | 标准系统（Linux 内核） |
|---|---|---|---|
| 典型 SoC | Hi3861、STM32 类 Cortex-M / RISC-V MCU | 带 MMU 的 Cortex-A（如 Hi3516 类） | RK3568 / RK3588 等应用处理器 |
| 内存管理 | 无 MMU，单一地址空间，可用 MPU 做保护 | 有 MMU，进程独立虚拟地址空间 | 有 MMU，完整虚拟内存 |
| 调度单位 | 任务（线程），无进程概念 | 进程 + 线程，类 POSIX | 进程 + 线程，完整 POSIX |
| 实时性 | 硬实时量级（中断/任务切换微秒级） | 较好（可配置） | 通用调度，默认非硬实时 |
| 文件系统 | 可选（LittleFS/FAT 等小型文件系统） | VFS，多种文件系统 | 完整文件系统 |
| 应用语言 | C/C++（SDK 封装） | C/C++ | C/C++、ArkTS/ArkUI |
| 启动时间 | 毫秒到百毫秒量级 | 秒级 | 数秒到数十秒 |

对本项目的含义：**Hi3861 这类无 MMU 的 MCU 上，应用与内核同在一个地址空间，一个任务写坏内存可能拖垮整机**，所以任务栈大小、缓冲区边界、指针检查必须自己负责（这也是 P4 的 JSON 缓冲区要有长度检查的原因）。RK3588 上是进程隔离，视觉进程崩溃不会拖垮系统，用 systemd 即可自动重启。

> 说明：以上内核特性为公开资料的概括。具体 LTS 版本号（如 3.x LTS 系）、可选组件以 SDK 版本为准，面试时不必背版本号。

### 2.6 LiteOS-M 要点：任务、定时器、消息队列

轻量系统常通过 **CMSIS-RTOS2 风格接口**使用内核对象（接口名以 SDK 头文件为准，下面是通用知识，不是 P4 源码）：

| 内核对象 | 典型接口 | 要点 |
|---|---|---|
| 任务（thread） | `osThreadNew(func, arg, &attr)`，attr 含名称、栈大小、优先级 | 栈大小要为最深调用链留余量；Wi-Fi/socket 相关任务栈通常需要几 KB |
| 延时 | `osDelay(ticks)` | 单位是系统 tick（典型配置 10 ms 一个 tick，以 SDK 配置为准），不是毫秒 |
| 软件定时器 | `osTimerNew(cb, osTimerPeriodic, arg, NULL)`，`osTimerStart(id, ticks)` | 回调在定时器任务上下文中执行，不能长时间阻塞，也不要做 socket 收发 |
| 消息队列 | `osMessageQueueNew(count, size, NULL)`，`Put/Get`（可带超时） | 任务间传数据的首选；消息按值拷贝，队列深度与消息大小共同决定占用内存 |
| 互斥锁 | `osMutexNew`，`Acquire/Release` | 保护共享变量；持锁时不要做耗时或阻塞操作 |
| 信号量/事件 | `osSemaphoreNew`、事件标志 | 中断到任务的通知；不要在中断里做重活 |

**手算：tick 与周期**。若 tick 为 10 ms，则 200 ms 采样周期是 20 ticks，1 s 状态机周期是 100 ticks；`osDelay(1)` 实际延时的下限是 1 个 tick（10 ms），**不可能做到 1 ms**。所以 P4 的"每 200 ms 采样、每 1 s 运行状态机"这类百毫秒、秒级周期用 tick 延时完全够用，但不要拿它做微秒级时序。

**一种合理的任务划分（教学示例，不代表源码结构）**：

```text
sensor_task  : 每 200 ms 读 ADC，5 次取中位数 -> 每 1 s 产出一个湿度值 -> 放入消息队列
control_task : 每 1 s 从队列取最新值，跑滞回状态机，写 GPIO，更新泵/模式状态（互斥保护）
net_task     : 维护 Wi-Fi/TCP 连接；每 2 s 打包传感 JSON 上报；接收命令并转交 control_task
uart_task    : 解析离线语音模块的固定命令词，置位手动/自动标志
```

要点：**控制任务不依赖网络任务**。网络任务阻塞或断线时，传感与控制任务照常运行，这正是"断网也能灌溉"的实现基础（对应 CV 的 "stable local closed loop"）。

**优先级反转**：低优先级任务持有互斥锁时，高优先级任务等待，中优先级任务又抢占低优先级任务，高优先级任务被间接拖住。RTOS 常用**优先级继承**缓解；设计上更稳妥的办法是缩短临界区、少用共享变量、用消息队列传值。

---

## 3. Hi3861

### 3.1 它是什么

Hi3861 是海思（HiSilicon）面向 IoT 的 **2.4 GHz Wi-Fi SoC**（公开资料：32 位 RISC-V 内核，主频约 160 MHz，约 352 KB SRAM、288 KB ROM、2 MB Flash，支持 802.11 b/g/n）。常见开发板（如 Hi3861 开发套件）带有 GPIO、ADC、I2C、SPI、UART、PWM 等外设引出。

> **面试表述**："Hi3861 is a low-cost Wi-Fi MCU with a RISC-V core and a few hundred kilobytes of RAM. It's good at sensing, GPIO control and talking over Wi-Fi, but not at heavy compute."

### 3.2 能力边界一览

| 维度 | 量级 | 对 P4 的含义 |
|---|---|---|
| 主频 | ~160 MHz | 足够做 ADC 采样、阈值判断、JSON 打包 |
| SRAM | ~352 KB | 不能缓存图像、不能放大模型；JSON 缓冲要小心 |
| Flash | 2 MB | 固件 + 少量配置 |
| 浮点 | 通常无 FPU 或性能弱（以数据手册为准） | 湿度标定用整数/少量浮点，别写重数学 |
| 网络 | Wi-Fi STA/AP，lwIP TCP/UDP | 与服务端用 TCP socket |
| 外设 | ADC、GPIO、I2C、SPI、UART、PWM | 读土壤探头、驱动继电器、语音模块 |
| 功耗 | 低 | 适合常开节点 |
| 成本 | 低 | 一个盆栽一个节点 |

### 3.3 外设在 P4 里的用法

- **ADC**：读土壤湿度探头的模拟电压。本项目取值：Hi3861V100 开发板（BearPi-HM Nano 同款引脚），ADC_CHANNEL_5（GPIO11），12 位（0–4095），参考 3.3 V 量程。→ [03 ADC 采集与标定](./03_Hi3861本地灌溉_ADC标定与传感策略.md)
- **GPIO**：输出控制继电器模块（**低电平触发**）；输入读语音识别模块的触发信号（若使用）。→ [04 滞回状态机、继电器与手动优先](./04_Hi3861滞回状态机_继电器与手动优先.md)
- **Wi-Fi + socket**：连接局域网，TCP 上报传感 JSON、接收命令。→ [08 TCP 通信](./08_TCP通信与协议设计.md)

### 3.3.1 外设逐项要点（通用知识 + 本项目取值）

| 外设 | 通用要点 | 本项目取值与注意 |
|---|---|---|
| ADC | SAR 型逐次逼近；分辨率 $N$ 位对应 $2^N$ 级；LSB $=V_{\text{ref}}/2^N$；输入阻抗有限，高阻源需要缓冲或更长采样时间 | 12 位、3.3 V：LSB $=3.3/4096\approx0.806$ mV；电容式探头输出为模拟电压，干燥空气读数约 2950，浸水约 950；5 次取中位数抗毛刺；厂商文档里的衰减/参考电压配置可能与此不同，以实机为准 |
| GPIO | 方向（输入/输出）、上下拉、电平；输出需先设方向再写值；上电默认状态由复位后的配置决定 | 继电器为低电平触发：低=吸合=泵开；**默认上电输出高**，避免上电即开泵；语音模块触发信号可作为输入读取 |
| I2C | 两线（SCL/SDA）开漏，需上拉；7 位地址；标准 100 kbit/s、快速 400 kbit/s；先写寄存器地址再读 | 同板挂 AHT20 空气温湿度传感器（器件地址 0x38 一类，以数据手册为准）；读取需要触发测量、等待数十毫秒再读 6 字节；总线忙或 NACK 要有超时与重试 |
| UART | 异步串口，需双方波特率/数据位/停止位一致；常用 8N1；无流控时要防止缓冲溢出 | 与离线语音识别模块通信（固定命令词“打开浇水/关闭浇水/自动模式”）；波特率按模块手册，常见 9600；调试日志另用一路 UART（常见 115200） |
| Wi-Fi STA | 2.4 GHz，802.11 b/g/n；STA 模式连路由器，经 DHCP 拿 IP，再用 lwIP socket 通信 | 连接顺序：启用 Wi-Fi，添加热点配置（SSID/密码），连接，等待 DHCP，建立 TCP；断线需重连与重建 socket |
| 光照 | 光敏电阻读 ADC，或 BH1750 经 I2C 读照度（lux） | 同板挂光敏/BH1750，上报字段 `lux` |

**Wi-Fi STA 连接流程（通用）**：

```text
EnableWifi()
 -> AddDeviceConfig(ssid, psk, security)    # 保存网络配置，得到 netId
 -> ConnectTo(netId)
 -> 等待连接事件 / 轮询链路状态
 -> 启动 DHCP 客户端，等待获得 IP（通常 1-5 s）
 -> socket(AF_INET, SOCK_STREAM) -> connect(server:8001)
 -> 周期 send(JSON)；select/recv 带超时读命令
 -> 任一步失败：关闭 socket，退避重连（如 1 s、2 s、4 s，上限若干十秒）
```

**手算：上报流量**。一条典型传感 JSON（字段 soil、raw、temp、hum、lux、pump、mode、seq）约 87 字节；本项目每 2 s 上报一次，即约 43.5 B/s、约 348 bit/s。这个量级对 Wi-Fi 毫无压力，瓶颈从来不是带宽，而是**连接可靠性**与**成帧**（见第 08 章）。

### 3.4 开发流程（典型）

1. PC 上安装 OpenHarmony 轻量系统的编译环境（常见：hb 构建工具、交叉编译工具链）；
2. 在 `applications/sample/...` 之类目录下写应用（`BUILD.gn` 描述构建，`main()`/入口函数里创建任务）；
3. 编译出固件，通过串口烧录工具烧到板子；
4. 串口日志（UART 打印）调试；
5. 联网：配置 SSID/密码，STA 模式连路由器，拿到 IP 后用 socket 连服务端。

具体目录、构建命令、SDK 版本是实机细节，面试时不必背；能讲出**"写任务、配 GPIO/ADC、连 Wi-Fi、开 socket、串口调试"**这条主线即可。启动与部署的完整顺序见第 6 节。

### 3.5 为什么一个盆栽一个 Hi3861 也合理

- 土壤湿度是**每个花盆各自的物理量**，泵也是每个盆/每路各自的执行器；
- 节点可以独立失效，不拖累其它节点；
- 成本极低，易于复制（这是 IoT 节点的普遍设计）。

### 3.6 资源预算手算：Hi3861 能放什么

| 项 | 数值 | 计算 | 结论 |
|---|---|---|---|
| SRAM 总量 | 352 KB | $352\times1024=360{,}448$ B | 固件运行时的栈、堆、协议栈缓冲都在这里 |
| 一条传感 JSON | 约 87 B | 8 个字段的紧凑 JSON | 256 B 的发送缓冲足够，要有长度检查 |
| 一层特征图（80×80×64，INT8） | 409,600 B | $80\times80\times64$ | 比整个 SRAM 还大（约 1.14 倍） |
| YOLOv5s INT8 权重 | 约 7.0 MB | $7.0\times10^6\times1$ B | 是 2 MB Flash 的 3.5 倍，放不下 |
| YOLOv5s FP32 权重 | 约 28 MB | $7.0\times10^6\times4$ B | 更放不下 |
| 一帧 640×640×3 输入 | 1,228,800 B | $640\times640\times3$ | 单是输入就是 SRAM 的 3.4 倍 |

结论是三重否定：**算力不够（5.1）、内存不够（SRAM 与 Flash 都不够）、没有摄像头与 NPU 接口**。所以分工不是偏好，而是资源约束的结果。

**一个合理的 SRAM 使用示意**（教学估算，非实测）：Wi-Fi 与 lwIP 协议栈占用大头；每个任务栈按需分配（例如控制任务 2-4 KB、网络任务 4-8 KB）；JSON 收发缓冲各数百字节。设计原则是**所有缓冲都有上限、所有拷贝都带长度**，因为无 MMU 的 MCU 上越界写不会触发段错误，而是悄悄破坏相邻数据。

---

## 4. RK3588

### 4.1 它是什么

RK3588 是瑞芯微（Rockchip）的高性能 SoC（公开资料：4×Cortex-A76 + 4×Cortex-A55 的 8 核 CPU、内置 NPU 约 6 TOPS（INT8）、多路 MIPI-CSI/USB3/PCIe/以太网、硬件视频编解码）。常以开发板形态出现（如 Orange Pi 5 Plus、Rock 5 等同系列板子）。

> 本项目取值：Orange Pi 5 Plus（RK3588，8 GB 内存），Ubuntu 22.04 Rockchip 镜像（内核 5.10）。

### 4.2 CPU / NPU / GPU 的分工

| 单元 | 擅长 | 在 P4 视觉链中的角色 |
|---|---|---|
| CPU | 控制流、通用计算、系统调用 | V4L2 取帧、颜色转换/缩放（可用 RGA 硬件加速）、后处理、TCP |
| NPU | 大规模并行的整数/低精度乘加（卷积） | 运行 RKNN 格式的 YOLOv5 网络 |
| RGA（2D 加速器） | 缩放、旋转、色彩空间转换 | 可选：减轻 CPU 的预处理负担；本项目未使用 RGA，预处理（letterbox、颜色转换）在 CPU 上做 |
| ISP / VPU | 图像信号处理、视频编解码 | 摄像头为 USB UVC 的 MJPEG 流，由 OpenCV 在 CPU 上软解（每帧约 4–6 ms），未用 VPU 硬解；USB 摄像头也不经过 ISP |

### 4.3 RKNN 工具链：两端分工

```text
PC（x86 Linux，训练好的 ONNX 权重）               板端（RK3588，aarch64）
┌──────────────────────────────┐                 ┌─────────────────────────────┐
│ RKNN-Toolkit2                │   model.rknn    │ RKNPU2 运行库 (librknnrt)    │
│  config → load_onnx → build  │ ─────────────→  │  rknn_init / rknn_inputs_set │
│  (量化、校准、图优化)         │                 │  rknn_run / rknn_outputs_get │
│  可在 PC 上用模拟器验证精度   │                 │  或 RKNN-Toolkit-Lite2(Python)│
└──────────────────────────────┘                 └─────────────────────────────┘
```

- **转换在 PC 做**：[07 INT8 量化与 RKNN 转换](./07_INT8量化与RKNN转换.md)；
- **推理在板端做**：[05](./05_RK3588边缘视觉_V4L2取帧与预处理.md)、[06](./06_YOLOv5输出解码_NMS与结构化事件.md)；
- **板端 C/C++ API**：`rknn_init → rknn_inputs_set → rknn_run → rknn_outputs_get → rknn_outputs_release → rknn_destroy`（API 名称以官方文档为准，面试说"the RKNN C API"即可）。

### 4.4 为什么需要 NPU 而不是只用 CPU

- YOLO 的主体是**卷积**，本质是大量乘加；
- CPU 通用但并行乘加吞吐有限，且功耗高；
- NPU 为 INT8 乘加阵列优化，同样的功耗下吞吐高一个量级以上（具体倍数取决于模型与实现，**不给具体数字**，除非你实测过）。

### 4.5 NPU 细节：多核、算子与精度

- **核数与算力**：RK3588 的 NPU 公开资料标称合计约 6 TOPS（INT8），由 3 个核心组成，每核约 2 TOPS。RKNN 运行库可以指定使用单核或多核（通过运行时初始化时的核心掩码选项，名称以 RKNN-Toolkit2/RKNPU2 文档为准）。本项目取值为**单核** INT8 推理，所以理论下界应按单核 2 TOPS 算（见 5.2 节）。
- **精度类型**：NPU 擅长 INT8；FP16 也支持但吞吐更低；FP32 通常不是它的强项。这就是模型要做 INT8 量化的原因（第 07 章）。
- **算子支持**：不是所有 ONNX 算子都能跑在 NPU 上，不支持的会回退 CPU（变慢）或转换失败。YOLOv5 的常规卷积、激活、上采样、拼接一般都支持；解码的 sigmoid/网格计算本项目放在 CPU 上（第 06、07 章）。
- **输入输出**：输入是 uint8 张量，本项目送 640×640×3 的 uint8 RGB；输出是 3 个检测头张量（80/40/20 三个尺度）。
- **并发**：NPU 按队列处理请求，多线程并发推理不会线性加速，反而增加排队。

### 4.6 RGA：2D 加速器

RGA 是独立于 CPU 的 2D 图形硬件，可做缩放、裁剪、旋转、格式转换（如 YUV 转 RGB）。把 letterbox 的缩放与颜色转换交给 RGA，可以节省 CPU 时间，并减少内存拷贝。**本项目没有用 RGA**，原因是 CPU 余量足够：见 5.5 节手算，仅 MJPEG 软解就只占一个核心的约 6%–9%。面试时可以这样说："RGA is a possible optimisation for the preprocessing, but the CPU had enough headroom, so I kept the pipeline simple."（译：RGA 是可能的优化项，但 CPU 余量足够，所以保持流水线简单。）

### 4.7 VPU：视频编解码

VPU 提供 H.264/H.265/MJPEG/JPEG 等硬件编解码（通过 Rockchip 的 MPP 库使用）。当摄像头是 MJPEG 且帧率高、分辨率大时，硬解可以显著降低 CPU 占用。本项目是 1280×720@30 的 MJPEG，OpenCV(V4L2) 取帧并在 CPU 软解，每帧约 4–6 ms；推理链稳定在约 15 FPS，所以每秒只需处理约 15 帧，其余帧被丢弃（见 5.5 节）。

### 4.8 内存带宽：容易被忽视的瓶颈

RK3588 采用 LPDDR4X/LPDDR5 内存，64 位总线。以 LPDDR4X-4224 为例，理论峰值带宽：

$$4224\times10^{6}\ \text{T/s}\times 8\ \text{B}=33.8\ \text{GB/s}$$

CPU、NPU、GPU、VPU、显示等共享这一带宽，实际可用远低于理论值。本项目的视觉流量并不大：

| 数据 | 大小 | 15 FPS 下的流量 |
|---|---|---|
| MJPEG 压缩帧（约 150 KB，教学估算） | 150 KB | 约 2.3 MB/s |
| 解码后 BGR 1280×720×3 | 2.76 MB | 约 41.5 MB/s |
| NPU 输入 640×640×3 uint8 | 1.23 MB | 约 18.4 MB/s |
| INT8 权重（每帧至少读一遍，约 8 MB） | 8 MB | 约 120 MB/s |

这些合计约 180 MB/s，远小于 33.8 GB/s（不到 1%）。因此本项目的瓶颈**不在内存带宽**，而在 NPU 利用率与 CPU 后处理。内存带宽成为瓶颈的典型情形是：高分辨率多路视频、中间激活很大、或多个模型同时跑。**这是估算，不是实测**；真实的带宽占用还包括中间特征图的读写，这也是 5.2 节实测利用率低于理想值的原因之一。

---

## 5. 为什么这样分工：一组手算

### 5.1 手算 A：YOLO 单帧的计算量 vs Hi3861

**已知（公开数据）**：YOLOv5s 在 640×640 输入下约 **16.5 GFLOPs**（官方 README 给出的量级；若你用的是别的规格，如 n/m/l，请替换）。

符号：FLOPs = 浮点运算次数；1 次乘加（MAC）≈ 2 FLOPs。

$$\text{MACs} \approx \frac{16.5\times 10^{9}}{2} = 8.25 \times 10^{9}$$

**Hi3861 的粗略上界（教学估算，非实测）**：假设 160 MHz、每周期理想完成 1 次 MAC（实际 MCU 做 8 位乘加通常更慢，且无 SIMD），则每秒最多：

$$160\times 10^{6} \ \text{MAC/s}$$

单帧至少需要：

$$t = \frac{8.25\times 10^{9}}{1.6\times 10^{8}} \approx 51.6\ \text{s}$$

结论：**即使理想条件下一帧也要近一分钟**，而且 352 KB 的 SRAM 连一层特征图（如 $80\times80\times 64$ 个 INT8 = 409,600 B ≈ 400 KB）都放不下。所以视觉推理**不可能**放在 Hi3861 上。

> 验算：$80\times 80 = 6400$，$6400\times 64 = 409{,}600$ 字节，大于 352 KB（352×1024 = 360,448 B）。

### 5.2 手算 B：NPU 的理论下界

RK3588 NPU 标称 6 TOPS（INT8；1 次乘加 = 2 次运算）：

$$t_{\min} \approx \frac{16.5\times 10^{9}\ \text{ops}}{6\times 10^{12}\ \text{ops/s}} = 2.75\ \text{ms}$$

这是**完全理想**（100% 利用率、无内存瓶颈）的下界。真实延迟会高得多——内存带宽、算子支持、层间数据搬运都会吃掉大部分时间。所以：

> **不要在面试里说"2.75 ms"当成你的推理时间**。本项目取值：NPU 单帧推理约 28 ms（单核，INT8，输入 640），整条链端到端约 60 ms，稳定约 15 FPS；测法是用 time.perf_counter 在各阶段打点，取连续 300 帧的均值与 p95（p95 约 75 ms）。

**手算：理论下界与实测的差距**（以单核 2 TOPS 为口径，因为本项目是单核推理）：

$$t_{\min,1\text{core}}=\frac{16.5\times10^{9}}{2\times10^{12}}=8.25\ \text{ms}$$

$$\text{利用率}=\frac{8.25\ \text{ms}}{28\ \text{ms}}\approx 29.5\%$$

即实测约 28 ms 对应有效算力约 589 GOPS，是单核标称的约 29.5%（相对整颗 6 TOPS 则约 9.8%）。差距来自：层间数据搬运、不同算子的利用率不一、权重与特征图的内存访问、输入输出拷贝，以及标称算力本身是峰值。面试可以说 "28 ms is about three to four times the ideal single-core bound, which is normal because of memory traffic and operator efficiency."（译：28 ms 约是单核理想下界的三到四倍，这是正常的。）

### 5.3 手算 C：视频流上传的带宽

教学示例：720p、30 fps、YUYV（每像素 2 字节）：

$$1280\times 720\times 2 = 1{,}843{,}200\ \text{B/帧}$$

$$1{,}843{,}200\times 30 = 55{,}296{,}000\ \text{B/s} \approx 55.3\ \text{MB/s} \approx 442\ \text{Mbit/s}$$

这远超 2.4 GHz Wi-Fi 的实际吞吐（常见几十 Mbit/s 量级），所以"把原始视频传给服务端"不可行。即使压成 MJPEG（每帧约 100–200 KB，教学估算）也要 3–6 MB/s；而 P4 只发**观测 JSON（几百字节）**加偶尔的 JPEG，带宽压力小两到三个数量级。

### 5.4 分工结论表

| 问题 | 谁最合适 | 理由 |
|---|---|---|
| 土干了就浇，湿了就停 | Hi3861 | 实时、近物理、断网要能工作 |
| 看叶子有没有病斑 | RK3588 | 需要 NPU 与摄像头接口 |
| 综合土壤、病斑、历史，给出建议并和用户交互 | 服务端 | 需要汇聚、LLM、Web |
| 人工开关 | 三层都要体现 | 手动优先从 UI 一路贯穿到 GPIO |


### 5.5 手算 D：端到端预算与 CPU 余量

端到端约 60 ms 由若干阶段组成。取值表只给出 NPU 约 28 ms、MJPEG 软解每帧约 4–6 ms、端到端约 60 ms；其余阶段按**教学分解**（非实测，只为说明预算怎么分）：

| 阶段 | 耗时（ms） | 来源 |
|---|---|---|
| 取帧 + MJPEG 软解 | 5 | 4–6 ms 的中值 |
| letterbox + BGR→RGB | 4 | 教学分解 |
| NPU 推理 | 28 | 本项目取值 |
| 解码 + NMS（CPU） | 15 | 教学分解 |
| 事件生成 + 打包 + 发送 | 8 | 教学分解 |
| 合计 | 60 | 与端到端取值一致 |

由此可得：

- **模型耗时只占端到端的 $28/60\approx46.7\%$**。把 NPU 再提速一倍，端到端也只降到约 46 ms；CPU 阶段同样重要。
- **吞吐 vs 延迟**：流水线若串行，吞吐上限 $1/60\ \text{ms}\approx16.7$ FPS，与稳定约 15 FPS 吻合；摄像头 30 FPS 时每帧间隔 33.3 ms，系统处理不过来，所以**约一半的帧会被丢弃**（只取最新帧，保证新鲜度）。
- **p95 与均值**：p95 约 75 ms，均值约 60 ms，尾部延迟比均值高约 25%，来自调度抖动与后处理候选框数量的波动。
- **CPU 余量**：MJPEG 软解每帧 4–6 ms、每秒约 15 帧，占用 $4\times15=60$ ms/s 到 $6\times15=90$ ms/s，即一个核心的 6%–9%；RK3588 有 8 个核心，所以不用 RGA/VPU 也完全可行。

### 5.6 用 Python 复核全部手算

下面的脚本复现本章所有手算（可直接运行，无第三方依赖）：

```python
mac = 16.5e9 / 2                      # YOLOv5s 约 16.5 GFLOPs，1 MAC = 2 FLOPs
print("MACs", mac)                    # 8.25e9
for ipc in (1.0, 0.25, 0.1):          # Hi3861 每周期完成的 MAC 数（理想/保守）
    print("Hi3861 s/frame @", ipc, "MAC/cycle:", mac / (160e6 * ipc))
print("SRAM B", 352 * 1024, "feature map B", 80 * 80 * 64)   # 360448 vs 409600
print("weights MB int8/fp32", 7.0, 28.0, "flash MB", 2)
print("NPU ideal ms 6T / 2T:", 16.5e9 / 6e12 * 1e3, 16.5e9 / 2e12 * 1e3)
print("util single core", 16.5e9 / 0.028 / 2e12)             # 约 0.295
print("YUYV 720p30 MB/s, Mbit/s:", 1280*720*2*30/1e6, 1280*720*2*30*8/1e6)
print("DDR GB/s", 4224e6 * 8 / 1e9)                           # 33.792
print("decode CPU load", 4 * 15 / 1000, 6 * 15 / 1000)        # 0.06, 0.09
print("model share of e2e", 28 / 60)                         # 0.467
```

输出要点：51.56 s / 206.25 s / 515.6 s（Hi3861 每帧，取决于每周期 MAC 数）、8.25 ms 与 2.75 ms（NPU 下界）、利用率 0.295、442.4 Mbit/s、33.8 GB/s、CPU 占用 6%–9%、模型占端到端 46.7%。

---

## 6. 启动与部署流程

### 6.1 总流程

```text
[PC]  训练/获取 YOLOv5 权重(.pt) → 导出 ONNX → RKNN-Toolkit2 量化 → .rknn
[RK3588] 部署 .rknn + 推理程序(C++/Python) → 摄像头 → 观测 → TCP → 服务端
[Hi3861] OpenHarmony 轻量系统固件 → ADC/GPIO/Wi-Fi → 状态机 → TCP
[服务端] Python: TCP 接收线程 + asyncio WebSocket + LangGraph → Web 前端
```

**关于"训练"**：训练在 PC GPU 上完成，不在 RK3588 上。本项目取值：自采约 1800 张叶片图像，在公开 COCO 预训练权重上微调 YOLOv5s 100 个 epoch；板端只运行转换后的 INT8 .rknn（约 8 MB）。详见第 06、07 章。

### 6.2 Hi3861 的构建、烧录与启动

1. **构建**：用 hb 之类的构建工具选择产品（板型），再编译；产物是固件镜像（`.bin`）。
2. **烧录**：通过 USB 转串口，使用烧录工具（如 HiBurn）选择串口与固件文件，按提示复位板子进入下载模式；烧录完成后复位运行。
3. **启动顺序**（概念）：Boot ROM，引导加载，内核初始化（LiteOS-M），驱动与服务初始化，应用入口创建任务。
4. **应用启动顺序**（教学示例）：

```text
初始化 GPIO：继电器引脚先设为输出并拉高（泵关），再做其它初始化   # 防止上电瞬间泵动作
初始化 ADC、I2C、UART
创建 sensor_task / control_task / uart_task          # 本地灌溉先于联网就绪
创建 net_task：连接 Wi-Fi -> DHCP -> TCP -> 上报
```

关键点：**先把执行器置于安全状态，再初始化其余外设；本地控制任务不等待网络**。

5. **调试**：串口日志（常见 115200，8N1）；上线前用万用表检查继电器输入脚电平。

### 6.3 RK3588 的部署

1. 刷写 Ubuntu 22.04 Rockchip 镜像，确认内核版本（5.10）与 NPU 驱动版本。
2. 安装 RKNN 运行库 librknnrt（1.6.0），其版本应与转换用的 RKNN-Toolkit2（1.6.0）匹配；版本不匹配会出现加载失败或结果异常。
3. 把 `.rknn` 与推理程序拷到板上；确认摄像头设备 `/dev/video0` 可读，格式为 MJPEG 1280×720@30。
4. 以 systemd 服务启动视觉进程，设置异常退出后自动重启；启动后先打印一次模型输入输出信息（形状、数据类型）作为自检。
5. 连接服务端 8002 端口，按 RK 帧格式发送（8 字节头 + JSON + 可选 JPEG）。

### 6.4 整体启动顺序与依赖

| 顺序 | 组件 | 依赖 | 失败时的行为 |
|---|---|---|---|
| 1 | 服务端（8000/8001/8002/8765） | 无 | 其它组件重连等待 |
| 2 | Hi3861 | 供电；Wi-Fi 与服务端非必需 | 本地灌溉独立运行，联网成功后再上报 |
| 3 | RK3588 视觉进程 | 摄像头、模型 | 摄像头缺失则报告并退避重试；不影响灌溉 |
| 4 | 网页 | 服务端 WebSocket | 断线后重连并拉全量快照 |

注意：**启动顺序不应成为正确性前提**。任何组件先起或后起，都应该能自动连上并恢复，这是用重连与退避实现的（第 08、12 章）。

---

## 7. 常见故障与排查

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| 上电泵立刻转 | 继电器低触发而默认输出低 | 万用表测输入脚；默认输出改高，逻辑与电平分离 |
| 湿度读数一直满量程或为 0 | 探头未接、接触不良、ADC 通道错 | 看 raw 是否越过有效范围（>3250 或 <200）；换通道对照 |
| 读数跳动大 | 供电噪声、泵启动拉低电压、探头未插稳 | 中位数滤波；示波器看电源；泵单独供电并共地 |
| Wi-Fi 连不上 | 2.4 GHz 与 5 GHz 混淆、密码错、信号弱 | 串口日志看连接状态；确认路由器开启 2.4 GHz |
| TCP 间歇断开 | 路由器省电/NAT 超时、服务端重启 | 周期上报兼作心跳；断线后退避重连并重建 socket |
| 烧录失败 | 串口号错、未进入下载模式、波特率过高 | 重新复位并选对串口；降低烧录波特率 |
| 板子偶发复位 | 任务栈溢出、泵启动拉低电源、看门狗超时 | 加大栈并打印水位；电源分离；确认任务周期喂狗 |
| `.rknn` 加载失败 | 运行库与 Toolkit 版本不匹配、目标平台错 | 统一为 1.6.0；转换时指定 rk3588 |
| 框整体偏移 | letterbox 还原漏减 pad | 共用 (r, pad) 并做往返测试（第 05、06 章） |
| 推理很慢 | 未使用 NPU（回退 CPU）、算子不支持、CPU 降频 | 看分阶段耗时；检查是否有 CPU 回退；查看温控与频率 |
| 帧延迟越来越大 | 处理速度低于采集，缓冲排队 | 只取最新帧，丢弃旧帧；记录帧龄 |
| JSON 偶发解析失败 | TCP 粘包/半包 | 长度前缀或括号配对成帧，切块测试（第 08 章） |

> 排查的通用方法：**沿数据链逐个边界查**，即探头电压、ADC 读数、湿度值、状态机决策、GPIO 电平、继电器、泵；摄像头帧、张量、输出、框、事件、字节流、服务端解析。每个边界都能单独观测，才能快速定位。

---

## 8. 面试怎么问、怎么答

**Q1. What is OpenHarmony, and why did you use it?**

> "OpenHarmony is an open-source operating system family for all kinds of smart devices, from tiny microcontrollers to phones. In my project, the Hi3861 node runs its lightweight system on a LiteOS-M kernel, which gives me tasks, drivers and a Wi-Fi stack on a very small chip. I also used it because the project was built for the Huawei ICT Competition's OpenHarmony track."

（译：OpenHarmony 是面向各类智能设备的开源操作系统家族，从微控制器到手机。Hi3861 节点跑的是它的轻量系统（LiteOS-M 内核），在很小的芯片上提供任务、驱动和 Wi-Fi 栈；同时项目也是为华为 ICT 大赛 OpenHarmony 赛道做的。）
追问方向：LiteOS-M 与 FreeRTOS 的区别？→ 都是 RTOS 类内核；LiteOS-M 属于 OpenHarmony 体系，提供 CMSIS 接口，应用用的是 SDK 封装好的 API，不必纠结内核差异。

**Q2. Why not run everything on the RK3588?**

> "The RK3588 could run more, but the irrigation loop should be close to the actuator, simple, always on, and isolated from vision failures. A Hi3861 node costs little, uses little power, and keeps working if the vision board or the server is down. The RK3588 is used where it is really needed: camera input and NPU inference."

**Q3. What does the NPU do that the CPU can't?**

> "The NPU is a dedicated array for low-precision multiply-accumulate operations, which is what convolution layers need. For an INT8 model it gives much higher throughput per watt than the CPU. The CPU still does capture, pre- and post-processing, and networking."

**Q4. Can you run YOLO on the Hi3861? Show me with numbers.**

> "No. YOLOv5s at 640 input is about 16.5 GFLOPs, so roughly 8 billion multiply-accumulates. Even at an ideal one MAC per cycle at 160 MHz, that's about 50 seconds per frame, and the activations alone wouldn't fit in a few hundred kilobytes of SRAM. The INT8 weights are about 7 megabytes, more than three times the 2 megabytes of flash."

追问：那为什么不用 TinyML 的小模型？→ 可以，但病害识别需要足够分辨率和容量；P4 的设计目标是边缘 NPU + MCU 分工，而不是把一切压进 MCU。

**Q5. Is the Hi3861 firmware Linux?**

> "No. It's an RTOS-style lightweight system: a LiteOS-M kernel with a small set of drivers, and my application runs as tasks."

**Q6. Why does irrigation survive a network failure?**

> "Sampling and control run in their own tasks and talk through a message queue; the network task only reports and receives commands. If Wi-Fi or the server fails, the network task retries, but the sampling and control tasks keep running, so the pump logic doesn't depend on the network."

（译：采样与控制各自一个任务，经消息队列通信；网络任务只负责上报和收命令。网络失败时网络任务重试，采样与控制照常运行。）

**Q7. How fast is your NPU inference, and how did you measure it?**

> "In our setup the NPU takes about 28 milliseconds per frame on a single core with the INT8 model at 640 input. The whole chain, from frame capture to the structured event, takes about 60 milliseconds and runs at roughly 15 frames per second. I put time.perf_counter timers around each stage and averaged over 300 consecutive frames; the 95th percentile was about 75 milliseconds."

（译：本项目 NPU 单帧约 28 ms（单核、INT8、输入 640），端到端约 60 ms、约 15 FPS；分阶段打点，连续 300 帧取均值，p95 约 75 ms。）
追问：*How does this compare with the ideal bound?* → "With 2 TOPS per core, ideal is 8.25 ms, so we use about 30% of it; the rest is memory traffic and operator efficiency."

**Q8. Did you use the RGA or the hardware video decoder?**

> "No. The camera sends MJPEG at 1280 by 720, and OpenCV decodes it on the CPU in about 4 to 6 milliseconds per frame. At 15 frames per second that's under 10 percent of one core, so I kept the pipeline simple. RGA and the hardware decoder are possible optimisations."

**Q9. Where is the bottleneck: NPU, CPU or memory?**

> "Mostly the NPU and the CPU post-processing. The model accounts for about 47 percent of the end-to-end time, and the CPU stages the rest. Memory bandwidth isn't the bottleneck here: the video traffic is well under one percent of the theoretical bandwidth. That's an estimate, not a measurement."

**Q10. What would you check if the board resets randomly?**

> "Stack overflow first, since the MCU has no memory protection to catch it; then the power supply, because the pump can pull the rail down when it starts; then the watchdog. That's why the pump has its own supply with a common ground."

---

## 9. 要点速记与自测

### 9.1 要点速记

1. OpenHarmony 是开源 OS 家族；三类系统：轻量（LiteOS-M，无 MMU）、小型（LiteOS-A，有 MMU）、标准（Linux 内核等）。
2. LiteOS-M 常用任务、软件定时器、消息队列、互斥锁；tick 通常 10 ms，`osDelay` 以 tick 计；控制任务不依赖网络任务。
3. Hi3861：Wi-Fi MCU，RISC-V 约 160 MHz，352 KB SRAM，2 MB Flash；本项目用 ADC_CHANNEL_5（GPIO11）12 位 3.3 V、GPIO 控继电器、I2C 挂 AHT20、UART 接语音模块、Wi-Fi STA + TCP。
4. RK3588：8 核 CPU + 约 6 TOPS（3 核各约 2 TOPS）NPU；本项目 Orange Pi 5 Plus，Ubuntu 22.04，单核 INT8 推理，未用 RGA/VPU。
5. RKNN：PC 上 RKNN-Toolkit2（1.6.0）转换/量化，板端 librknnrt（1.6.0）推理。
6. 手算：YOLOv5s 约 8.25 G MAC；Hi3861 理想也要约 52 s/帧；权重 7 MB 大于 Flash 2 MB；特征图 409,600 B 大于 SRAM。
7. NPU 下界：6 TOPS 为 2.75 ms，单核 2 TOPS 为 8.25 ms；实测约 28 ms，单核利用率约 29.5%。
8. 端到端约 60 ms、约 15 FPS；模型占约 47%；p95 约 75 ms。
9. 原始 720p30 YUYV 约 442 Mbit/s，不可能经 Wi-Fi 上传；发观测而不是发视频。
10. 分工原则：近物理的放 MCU，吃算力的放 NPU，要汇聚/语言/界面的放服务端。

### 9.2 自测题

1. OpenHarmony 的三类系统分别对应什么设备？Hi3861 属于哪类、用什么内核？无 MMU 带来什么风险？
2. 给出 YOLO 不能放在 Hi3861 上的三个量化理由（算力、SRAM、Flash）。
3. RKNN-Toolkit2 与 RKNPU2 运行库分别在哪里运行、做什么？两者版本为什么要匹配？
4. 720p、30 fps、YUYV 原始流量是多少？为什么不能直接上传？
5. 为什么 2.75 ms 不能作为推理时间？单核口径的下界是多少？利用率约多少？
6. 若 tick 为 10 ms，200 ms 采样周期是多少个 tick？`osDelay(1)` 能做到 1 ms 吗？
7. 简述 Hi3861 联网的步骤，以及断线后应该怎么做。
8. 端到端 60 ms 里模型占多少比例？为什么不能只优化 NPU？
9. 无 MMU 的 Hi3861 与有进程隔离的 RK3588，在"内存越界"的后果上有什么不同？
10. 本项目为什么没有用 RGA？依据是什么计算？

**参考要点**：

1. 轻量/小型/标准；Hi3861 属轻量，LiteOS-M；越界写不会被捕获，可能破坏其它任务数据，需自己做边界检查。
2. 理想约 52 s/帧（实际更慢）；一层特征图 409,600 B 超过 360,448 B 的 SRAM；7 MB 权重超过 2 MB Flash。
3. 前者在 PC（x86 Linux）转换量化，后者在板端推理；版本不匹配会加载失败或结果异常。
4. 约 55.3 MB/s，约 442 Mbit/s，超出 Wi-Fi 实际吞吐。
5. 它是 100% 利用率的理想下界，未计内存带宽/算子/搬运；单核下界 8.25 ms，利用率约 29.5%（8.25/28）。
6. 20 个 tick；不能，下限是 1 个 tick（10 ms）。
7. 启用 Wi-Fi，添加配置，连接，DHCP，TCP；失败则关闭 socket 并退避重连，本地灌溉不受影响。
8. 28/60 约 46.7%；其余在 CPU 阶段和发送，优化要看整条链。
9. Hi3861 无 MMU，越界可悄悄破坏数据；RK3588 进程隔离，崩溃只影响本进程；因此 Hi3861 上缓冲区必须有长度检查。
10. 软解只占一个核心约 6%–9%，CPU 余量充足，没有必要引入额外复杂度。

## 相关链接

- 上一章：[01 系统总览与三层架构](./01_系统总览与三层架构.md)
- 下一章：[03 Hi3861 本地灌溉：ADC 采集、标定与传感失效策略](./03_Hi3861本地灌溉_ADC标定与传感策略.md)
- 基础：[OpenHarmony、Hi3861、RK3588 平台基础（基础篇）](../../02_基础知识/08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)
- 返回：[P4 README](./README.md)
