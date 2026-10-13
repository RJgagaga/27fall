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
6. 开发与部署流程速览
7. 面试怎么问、怎么答
8. 要点速记与自测

> **关于数字的说明**：本章的芯片规格来自厂商/社区的**公开资料**，面试时说"according to the public datasheet"；你自己板子的具体型号、内存、系统镜像属于**实机细节**，见 README 自查表。

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

> **注意**：RK3588 板端实际运行的是 **Linux 用户态程序**（V4L2 取帧、RKNN 运行库推理），还是 OpenHarmony 标准系统镜像，要以你的板端为准：**【待你补全：RK3588 上跑的是什么系统镜像与版本】**。无论哪种，V4L2 和 RKNN 的调用方式是相同的（它们都属于 Linux 内核/驱动层的接口）。面试中说 "the RK3588 runs the vision process in userspace on top of the Linux kernel interfaces (V4L2 and the RKNN runtime)" 是安全且准确的说法，前提是你确认了这点。

### 2.3 关键技术点（面试能说到这个深度就够）

- **内核抽象层（KAL）/ CMSIS 接口**：让同一份应用代码在不同内核上编译；轻量系统常用 CMSIS-RTOS2 风格的任务、互斥锁、消息队列 API。
- **HDF（Hardware Driver Foundation）**：驱动框架，把外设驱动做成"可配置、可复用"的模块；小型/标准系统用得较多。
- **分布式软总线（DSoftBus）与设备虚拟化**：让设备之间发现、认证、传输数据；是"全场景协同"的底座。（P4 没有用软总线，设备间是普通 Wi-Fi + TCP，不要在面试里夸大。）
- **应用开发**：轻量系统用 C/C++（本项目的 Hi3861 部分）；标准系统用 ArkTS/ArkUI。

### 2.4 在 P4 里的位置

- **Hi3861 的固件**：基于 OpenHarmony 轻量系统（LiteOS-M），任务、定时、GPIO/ADC 驱动接口、Wi-Fi、lwIP socket 都来自 SDK。
- **竞赛背景**：CV 写 "9th Huawei ICT Competition, Programming (OpenHarmony) — Third Prize, China National Final; First Prize, Provincial Round"，与 P4 相关。回答时只说 CV 有的事实，不要补充排名人数、评委评价等没依据的细节。

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

- **ADC**：读土壤湿度探头的模拟电压（通道号与分辨率以数据手册/板子为准）。→ [03 ADC 采集与标定](./03_Hi3861本地灌溉_ADC标定与传感策略.md)
- **GPIO**：输出控制继电器模块（**低电平触发**）；输入读语音识别模块的触发信号（若使用）。→ [04 滞回状态机、继电器与手动优先](./04_Hi3861滞回状态机_继电器与手动优先.md)
- **Wi-Fi + socket**：连接局域网，TCP 上报传感 JSON、接收命令。→ [08 TCP 通信](./08_TCP通信与协议设计.md)

### 3.4 开发流程（典型）

1. PC 上安装 OpenHarmony 轻量系统的编译环境（常见：hb 构建工具、交叉编译工具链）；
2. 在 `applications/sample/...` 之类目录下写应用（`BUILD.gn` 描述构建，`main()`/入口函数里创建任务）；
3. 编译出固件，通过串口烧录工具烧到板子；
4. 串口日志（UART 打印）调试；
5. 联网：配置 SSID/密码，STA 模式连路由器，拿到 IP 后用 socket 连服务端。

具体目录、构建命令、SDK 版本是实机细节，面试时不必背；能讲出**"写任务、配 GPIO/ADC、连 Wi-Fi、开 socket、串口调试"**这条主线即可。

### 3.5 为什么一个盆栽一个 Hi3861 也合理

- 土壤湿度是**每个花盆各自的物理量**，泵也是每个盆/每路各自的执行器；
- 节点可以独立失效，不拖累其它节点；
- 成本极低，易于复制（这是 IoT 节点的普遍设计）。

---

## 4. RK3588

### 4.1 它是什么

RK3588 是瑞芯微（Rockchip）的高性能 SoC（公开资料：4×Cortex-A76 + 4×Cortex-A55 的 8 核 CPU、内置 NPU 约 6 TOPS（INT8）、多路 MIPI-CSI/USB3/PCIe/以太网、硬件视频编解码）。常以开发板形态出现（如 Orange Pi 5、Rock 5 等同系列板子）。

> 你的具体板子型号是**【待你补全】**。

### 4.2 CPU / NPU / GPU 的分工

| 单元 | 擅长 | 在 P4 视觉链中的角色 |
|---|---|---|
| CPU | 控制流、通用计算、系统调用 | V4L2 取帧、颜色转换/缩放（可用 RGA 硬件加速）、后处理、TCP |
| NPU | 大规模并行的整数/低精度乘加（卷积） | 运行 RKNN 格式的 YOLOv5 网络 |
| RGA（2D 加速器） | 缩放、旋转、色彩空间转换 | 可选：减轻 CPU 的预处理负担（是否使用【待你补全】） |
| ISP / VPU | 图像信号处理、视频编解码 | 本项目可能用到 MJPEG 解码（软/硬【待你补全】） |

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

> **不要在面试里说"2.75 ms"当成你的推理时间**。你的实测推理时间/FPS 是实机细节：**【待你补全：板端单帧推理耗时、整条链端到端 FPS，附测量方法】**。

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

---

## 6. 开发与部署流程速览

```text
[PC]  训练/获取 YOLOv5 权重(.pt) → 导出 ONNX → RKNN-Toolkit2 量化 → .rknn
[RK3588] 部署 .rknn + 推理程序(C++/Python) → 摄像头 → 观测 → TCP → 服务端
[Hi3861] OpenHarmony 轻量系统固件 → ADC/GPIO/Wi-Fi → 状态机 → TCP
[服务端] Python: TCP 接收线程 + asyncio WebSocket + LangGraph → Web 前端
```

**关于"训练"**：CV 写的是 "built a YOLOv5/RKNN INT8 inference chain"（构建**推理链**）。模型训练的数据集、类别、轮数、mAP 等**不要凭空补充**；如果你确实训练过模型，请补进 README 自查表；如果用的是公开权重或他人权重，面试就如实说"I focused on the deployment and system integration side"。

---

## 7. 面试怎么问、怎么答

**Q1. What is OpenHarmony, and why did you use it?**

> "OpenHarmony is an open-source operating system family for all kinds of smart devices, from tiny microcontrollers to phones. In my project, the Hi3861 node runs its lightweight system on a LiteOS-M kernel, which gives me tasks, drivers and a Wi-Fi stack on a very small chip. I also used it because the project was built for the Huawei ICT Competition's OpenHarmony track."

（译：OpenHarmony 是面向各类智能设备的开源操作系统家族，从微控制器到手机。Hi3861 节点跑的是它的轻量系统（LiteOS-M 内核），在很小的芯片上提供任务、驱动和 Wi-Fi 栈；同时项目也是为华为 ICT 大赛 OpenHarmony 赛道做的。）
追问方向：LiteOS-M 与 FreeRTOS 的区别？→ 都是 RTOS 类内核；LiteOS-M 属于 OpenHarmony 体系，提供 CMSIS 接口，你用的是 SDK 封装好的 API，不必纠结内核差异。

**Q2. Why not run everything on the RK3588?**

> "The RK3588 could run more, but the irrigation loop should be close to the actuator, simple, always on, and isolated from vision failures. A Hi3861 node costs little, uses little power, and keeps working if the vision board or the server is down. The RK3588 is used where it is really needed: camera input and NPU inference."

**Q3. What does the NPU do that the CPU can't?**

> "The NPU is a dedicated array for low-precision multiply-accumulate operations, which is what convolution layers need. For an INT8 model it gives much higher throughput per watt than the CPU. The CPU still does capture, pre- and post-processing, and networking."

**Q4. Can you run YOLO on the Hi3861? Show me with numbers.**

> "No. YOLOv5s at 640 input is about 16.5 GFLOPs, so roughly 8 billion multiply-accumulates. Even at an ideal one MAC per cycle at 160 MHz, that's about 50 seconds per frame, and the activations alone wouldn't fit in a few hundred kilobytes of SRAM."

追问：那为什么不用 TinyML 的小模型？→ 可以，但病害识别需要足够分辨率和容量；P4 的设计目标是边缘 NPU + MCU 分工，而不是把一切压进 MCU。

**Q5. Is the Hi3861 firmware Linux?**

> "No. It's an RTOS-style lightweight system: a LiteOS-M kernel with a small set of drivers, and my application runs as tasks."

---

## 本章要点速记

1. OpenHarmony 是开源 OS 家族；三类系统：轻量（LiteOS-M）、小型、标准。
2. Hi3861：Wi-Fi MCU，RISC-V ~160 MHz，几百 KB SRAM，擅长 ADC/GPIO/联网，不擅长重计算。
3. RK3588：8 核 CPU + 约 6 TOPS NPU；视觉链里 CPU 做取帧/预处理/后处理，NPU 做卷积。
4. RKNN：PC 上 RKNN-Toolkit2 转换/量化，板端 RKNPU2 运行库推理。
5. 手算：YOLOv5s ≈ 8.25 G MAC；Hi3861 理想也要 ~50 s/帧；NPU 理论下界 ~2.75 ms（不要当实测）。
6. 原始 720p30 YUYV ≈ 442 Mbit/s，不可能经 Wi-Fi 上传；发观测而不是发视频。
7. 分工原则：近物理的放 MCU，吃算力的放 NPU，要汇聚/语言/界面的放服务端。
8. 型号、镜像、实测耗时都是实机细节，见 README 自查表。

## 自测题

1. OpenHarmony 的三类系统分别对应什么设备？Hi3861 属于哪类、用什么内核？
2. 给出 YOLO 不能放在 Hi3861 上的两个量化理由。
3. RKNN-Toolkit2 与 RKNPU2 运行库分别在哪里运行、做什么？
4. 720p、30 fps、YUYV 原始流量是多少？为什么不能直接上传？
5. 为什么 2.75 ms 不能作为你的推理时间？

**参考要点**：
1. 轻量/小型/标准；Hi3861 属轻量，LiteOS-M。
2. 计算量（~50 s/帧理想）与内存（特征图超过 SRAM）。
3. 前者在 PC（x86 Linux）转换量化，后者在板端推理。
4. ≈55.3 MB/s≈442 Mbit/s，超出 Wi-Fi 实际吞吐。
5. 它是 100% 利用率的理想下界，未计内存带宽/算子/搬运；实测要用计时。

## 相关链接

- 上一章：[01 系统总览与三层架构](./01_系统总览与三层架构.md)
- 下一章：[03 Hi3861 本地灌溉：ADC 采集、标定与传感失效策略](./03_Hi3861本地灌溉_ADC标定与传感策略.md)
- 基础：[OpenHarmony、Hi3861、RK3588 平台基础（基础篇）](../../02_基础知识/08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)
- 返回：[P4 README](./README.md)
