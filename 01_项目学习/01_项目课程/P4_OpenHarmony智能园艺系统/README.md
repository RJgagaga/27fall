# P4 OpenHarmony 智能园艺系统（边缘视觉 + Agent + 本地灌溉）

> **一页纸**：个人项目，2024.05–2024.08。Hi3861 做土壤湿度驱动的本地灌溉闭环（独立启停阈值、手动优先）；RK3588 用 YOLOv5/RKNN INT8 把摄像头帧变成带置信度与时间戳的结构化观测；服务端用 LangGraph 决策链（规则 fast path 处理极端状态）输出结构化建议或受约束的命令请求；系统把视觉观测、环境状态和用户命令统一为可追踪的系统状态，经 Web 交互与设备端灌溉形成闭环。相关奖项（以 CV 为准）：第九届华为 ICT 大赛编程（OpenHarmony）赛道全国总决赛三等奖、省赛一等奖。

## CV 原文（唯一口径）

> **OpenHarmony Smart Gardening System** — Personal project · Edge vision and autonomous control — May 2024 – Aug 2024
> - **Edge vision inference**: built a YOLOv5/RKNN INT8 inference chain on RK3588, converting camera frames into structured observations with confidence and timestamps.
> - **Agent decision chain**: built an environment and vision-event decision chain with LangGraph, with a rule-based fast path for extreme states, issuing structured recommendations or constrained requests to the execution layer.
> - **Local irrigation control**: implemented a soil-moisture-driven pump state machine on Hi3861 with independent start/stop thresholds and manual-priority override for a stable local closed loop.
> - **System integration**: unified visual observations, environment state and user commands into a traceable system state, orchestrated decisions with a server-side agent and closed the loop through web interaction and device-side irrigation.

## 一句话记忆

三层（Hi3861 / RK3588 / 服务端）、五类信息（传感读数、模型观测、建议、命令请求、设备反馈）、一条链：**建议 ≠ 命令请求 ≠ 设备收到 ≠ 设备已动作 ≠ 水已浇到**。

## 阅读顺序

| 序 | 文件 | 内容 | 对应 CV |
|---|---|---|---|
| 1 | [01 系统总览与三层架构](./01_系统总览与三层架构.md) | 三层、五类信息、数据链、四级权威 | 全部 |
| 2 | [02 平台基础](./02_平台基础_OpenHarmony_Hi3861_RK3588.md) | OpenHarmony / Hi3861 / RK3588、分工手算 | 全部 |
| 3 | [03 ADC 标定与传感策略](./03_Hi3861本地灌溉_ADC标定与传感策略.md) | ADC、两点标定、滤波、传感失效 | Local irrigation |
| 4 | [04 滞回状态机与手动优先](./04_Hi3861滞回状态机_继电器与手动优先.md) | 滞回、低触发 GPIO、手动优先、语音 | Local irrigation |
| 5 | [05 V4L2 取帧与预处理](./05_RK3588边缘视觉_V4L2取帧与预处理.md) | 缓冲队列、像素格式、letterbox | Edge vision |
| 6 | [06 YOLOv5 解码、NMS、结构化事件](./06_YOLOv5输出解码_NMS与结构化事件.md) | 输出张量、NMS、置信度、时间戳 | Edge vision |
| 7 | [07 INT8 量化与 RKNN 转换](./07_INT8量化与RKNN转换.md) | 量化公式、逐通道、校准、转换≠训练 | Edge vision |
| 8 | [08 TCP 通信与协议](./08_TCP通信与协议设计.md) | 字节流、长度前缀、JSON+JPEG | Integration |
| 9 | [09 服务端](./09_服务端_DataStore_WebSocket_asyncio.md) | DataStore、WebSocket、asyncio、跨线程 | Integration |
| 10 | [10 LangGraph 决策链](./10_LangGraph决策链.md) | 六节点、fast path、LLM 校验、记忆/RAG | Agent chain |
| 11 | [11 执行边界](./11_执行边界_白名单_幂等与设备反馈.md) | 白名单、人工优先、去重与幂等、ACK | Agent chain |
| 12 | [12 系统集成与可追踪状态](./12_系统集成与可追踪系统状态.md) | 统一状态、时间、trace、降级 | Integration |
| 13 | [13 故障与测试](./13_故障与测试.md) | 18 个故障、测试方案 | 全部 |
| 99 | [99 英文面试口述与深挖问答](./99_英文面试口述与深挖问答.md) | 30s/90s/3min、60 问、STAR | 面试 |

**最短路线（时间紧）**：01 → 04 → 06 → 07 → 10 → 11 → 99。

## 基础文档依赖

- 平台：[OpenHarmony、Hi3861、RK3588](../../02_基础知识/08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)；通信：[UART/I2C/TCP](../../02_基础知识/08_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)；MCU：[GPIO/中断](../../02_基础知识/08_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)
- 视觉：[YOLO](../../02_基础知识/07_感知与机器学习/03_卷积网络与目标检测_YOLO.md)、[量化与 RKNN](../../02_基础知识/07_感知与机器学习/04_模型量化与边缘部署_RKNN.md)、[LLM 与 LangGraph](../../02_基础知识/07_感知与机器学习/05_Transformer_LLM与Agent_LangGraph.md)
- 控制/时间：[反馈控制](../../02_基础知识/04_控制理论/01_反馈控制与系统响应基础.md)、[限幅与饱和](../../02_基础知识/04_控制理论/07_前馈_增益调度_限幅与饱和.md)、[时间同步与数据有效性](../../02_基础知识/05_状态估计/06_时间同步_坐标系一致性与数据有效性.md)
- 八股：[CV 与检测](../../03_八股知识/02_人工智能/03_计算机视觉与目标检测八股.md)、[部署与量化](../../03_八股知识/02_人工智能/06_模型部署与量化八股.md)、[Agent 与 RAG](../../03_八股知识/02_人工智能/07_Agent与RAG八股.md)、[网络](../../03_八股知识/01_计算机基础/03_计算机网络.md)
- 同构项目：[P6（proposal → policy-check → execution → feedback）](../P6_深开鸿M-Robots平台软件/README.md)；面试总览：[七个项目口述](../../04_面试准备/03_项目口述总览/01_七个项目30秒_90秒_3分钟口述.md)
- 学习笔记原始问答：[问答补充](../../05_学习笔记归档/P4/P4完整问答补充.txt)、[逐字稿](../../05_学习笔记归档/P4/P4_本对话问答逐字稿.md)、[逐字稿第一份](../../05_学习笔记归档/P4/P4_本对话问答逐字稿_第一份.md)

## 实机细节自查表

> 本项目源码目录是空子模块；以下细节 CV 与笔记均未给出确切值，面试官大概率会追问。请逐项补全真实情况（没有就写"未做/未测"，不要编）。

| 类别 | 项目 | 你的真实情况 |
|---|---|---|
| 视觉模型 | 训练数据集来源与规模 | 【待你补全】 |
| | 类别列表与数量 | 【待你补全】 |
| | 模型是你训练的还是现成权重 | 【待你补全】 |
| | YOLOv5 规格（n/s/m）、参数量、.rknn 大小 | 【待你补全】 |
| | mAP（FP32 / INT8）、Precision/Recall | 【待你补全】 |
| | 输入分辨率（模型 640×640 是否属实）、摄像头分辨率/帧率 | 【待你补全】 |
| | 预处理：letterbox 还是直接 resize；BGR/RGB | 【待你补全】 |
| | conf / NMS 阈值 | 【待你补全】 |
| 量化 | 校准集张数、来源、覆盖场景 | 【待你补全】 |
| | Toolkit 与运行库版本、逐通道/算法选项 | 【待你补全】 |
| | 输出头形态（融合/分头）、是否混合量化 | 【待你补全】 |
| 性能 | 单帧模型耗时、端到端 FPS 与延迟及测量方法 | 【待你补全】 |
| 平台 | RK3588 开发板型号与系统镜像 | 【待你补全】 |
| | Hi3861 开发板、ADC 通道/位数/参考电压 | 【待你补全】 |
| | 探头型号、继电器模块、泵参数、电源与隔离 | 【待你补全】 |
| 本地控制 | 实际启停阈值与比较符（笔记线索 56/65） | 【待你补全】 |
| | 标定方法与参考读数（笔记线索 −0.05、147.52） | 【待你补全】 |
| | 采样周期、滤波、最长开泵时间、传感失效处理 | 【待你补全】 |
| | 语音模块与触发逻辑 | 【待你补全】 |
| 协议 | 长度字段宽度/字节序/各段顺序 | 【待你补全】 |
| | Hi3861 上报格式、字段与周期 | 【待你补全】 |
| | 命令名称与设备端映射、是否有 ACK/action_id | 【待你补全】 |
| 服务端 | LLM 型号与调用方式、是否可产生命令请求 | 【待你补全】 |
| | fast path 实际条件与阈值 | 【待你补全】 |
| | 记忆/RAG 实际实现与是否启用 | 【待你补全】 |
| | WebSocket 库、新鲜度阈值、队列大小 | 【待你补全】 |
| 验证 | 实机验证到哪一级（V0–V5，见第 13 章）、真实踩过的 3 个故障 | 【待你补全】 |

## 口径备忘（仅自查，面试按 CV 说）

1. **fast path**：CV 写 "extreme states"；笔记中的示例把 fast path 阈值写成与本地启动阈值同为 56。教材按 CV 的"极端状态"讲，用教学阈值 20；你需核对实际条件。
2. **时间戳**：CV 说观测带时间戳；笔记称 Hi3861 传感 JSON 的 timestamp 当前为 0、RK 用设备系统时间。面试按"观测带采集时间 + 服务端接收时间"讲，不主动说传感时间戳为 0。
3. **预处理**：对话中以 letterbox 讲解，读源码印象为直接 resize；教材两种都讲并要求你核对。
4. **输出张量**：笔记按 YOLOv8 形态 (1,C,A) 讲；CV 是 YOLOv5，教材按 v5 讲。
5. **协议**：长度头有"8 字节"与"4 字节"两种印象；服务端对 Hi3861 流用括号配对。教材不写死具体字节数。
6. **接口风险**：此前静态阅读发现服务端动作枚举（start/stop）与设备端命令（water_on/water_off 等）之间未见适配，且设备远程开关可能要求手动模式。这是待核对风险，非已证实故障；面试按 CV 讲闭环，被追问时诚实说明验证到哪一级。
7. **语音 off 与手动**：语音关闭可能恢复自动，随后低湿会重开；不要说"手动 off 永久有效"。
8. **去重**：`event_id:scope` 约 60 s、分析前标记，只防重复分析，不是 exactly-once。
9. **训练**：`onnx2rknn.py` 只证明部署转换，不证明训练或 mAP/FPS。
10. **记忆/RAG**：笔记只确认 state 有字段，不能声称已实现长期记忆。
