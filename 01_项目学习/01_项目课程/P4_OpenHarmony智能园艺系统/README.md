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

> 本项目源码目录是空子模块。以下为教材统一取值，与实机不一致时以实机为准并同步修改正文。面试官大概率会追问这些细节。

| 类别 | 项目 | 教材统一取值 |
|---|---|---|
| 视觉模型 | 训练数据集来源与规模 | 自采约 1800 张植物叶片图像（盆栽场景，手机+摄像头，白天/阴天/补光），LabelImg 标注，8:2 划分训练/验证 |
| | 类别列表与数量 | 3 类：normal、wilting、disease_spot |
| | 模型是你训练的还是现成权重 | 自己微调：在公开 COCO 预训练权重上微调 100 epoch（训练在 PC GPU，不在 RK3588） |
| | YOLOv5 规格（n/s/m）、参数量、.rknn 大小 | YOLOv5s，参数量约 7.0 M，INT8 .rknn 约 8 MB |
| | mAP（FP32 / INT8）、Precision/Recall | mAP@0.5：FP32 0.86 / INT8 0.83；Precision 约 0.85；Recall：wilting 0.84→0.80，disease_spot 0.78→0.73，normal 0.93→0.92 |
| | 输入分辨率（模型 640×640 是否属实）、摄像头分辨率/帧率 | 模型输入 640×640；摄像头 USB UVC，MJPEG 1280×720@30 |
| | 预处理：letterbox 还是直接 resize；BGR/RGB | letterbox（等比缩放，114 灰边填充）；BGR→RGB，uint8 送 NPU；解码按 (r, pad) 还原框 |
| | conf / NMS 阈值 | obj_conf 0.25（类别置信×obj≥0.25 进候选）；NMS IoU 0.45，按类别；事件门：同类连续 3 帧 conf≥0.5，同类事件最短间隔 5 s |
| 量化 | 校准集张数、来源、覆盖场景 | 200 张（训练集随机 150 + 现场补光/夜间 50），覆盖白天/阴天/补光/夜间 |
| | Toolkit 与运行库版本、逐通道/算法选项 | RKNN-Toolkit2 1.6.0 + librknnrt 1.6.0；asymmetric_quantized-8，quantized_algorithm=normal，optimization_level=3，权重逐通道 |
| | 输出头形态（融合/分头）、是否混合量化 | 3 个检测头分别输出（80/40/20 三个张量，解码在 CPU）；未做混合量化 |
| 性能 | 单帧模型耗时、端到端 FPS 与延迟及测量方法 | NPU 单帧推理约 28 ms（单核，INT8）；端到端约 60 ms，稳定约 15 FPS；测法：time.perf_counter 分阶段打点，连续 300 帧取均值与 p95（p95≈75 ms） |
| 平台 | RK3588 开发板型号与系统镜像 | Orange Pi 5 Plus（RK3588，8 GB），Ubuntu 22.04 Rockchip 镜像（内核 5.10） |
| | Hi3861 开发板、ADC 通道/位数/参考电压 | Hi3861V100 开发板（BearPi-HM Nano 同款引脚），LiteOS-M 轻量系统；ADC 12 位（0–4095），参考 3.3 V，通道 ADC_CHANNEL_5（GPIO11） |
| | 探头型号、继电器模块、泵参数、电源与隔离 | 电容式土壤湿度探头（v1.2 类，模拟输出，3.3 V 供电）；1 路光耦隔离、低电平触发 5 V 继电器模块；5 V 微型潜水泵约 3 W，独立 5 V 电源并与 Hi3861 共地；泵并联续流二极管 |
| 本地控制 | 实际启停阈值与比较符（笔记线索 56/65） | Ton=56（m<56 开），Toff=65（m>65 关），严格比较，滞回带 9 |
| | 标定方法与参考读数（笔记线索 −0.05、147.52） | 两点法：空气约 2950→0%，浸水约 950→100%；m=−0.05·raw+147.52，clip[0,100]，转 int16；整型截断比较，停止条件实际落在 raw≤1630 |
| | 采样周期、滤波、最长开泵时间、传感失效处理 | 每 200 ms 采样、5 次中位数→每 1 s 一个湿度值；最长开泵 60 s 后强制关并冷却 120 s；raw>3250 或 <200 或连续 3 次读取失败→valid=false，关泵并上报 sensor_fault |
| | 语音模块与触发逻辑 | 离线语音识别模块（UART），固定命令词“打开浇水/关闭浇水/自动模式”；语音触发进入手动，“关闭”后恢复自动 |
| 协议 | 长度字段宽度/字节序/各段顺序 | Hi3861→服务端 8001 为紧凑 JSON、括号配对成帧；RK3588→服务端 8002 为二进制：[4 字节大端 JSON 长度 N][4 字节大端 JPEG 长度 M] 共 8 字节头 + JSON + JPEG（无图 M=0） |
| | Hi3861 上报格式、字段与周期 | 紧凑 JSON，字段 soil、raw、temp、hum、lux、pump、mode、seq（timestamp 为 0），每 2 s 上报 |
| | 命令名称与设备端映射、是否有 ACK/action_id | 服务端动作 start/stop 经适配层映射为 water_on/water_off/set_duration，JSON {"cmd","duration","action_id"}；设备回 ACK {"ack":action_id,"accepted","pump","mode"}，下一条传感上报带 pump 状态 |
| 服务端 | LLM 型号与调用方式、是否可产生命令请求 | OpenAI 兼容接口的 DeepSeek-V3 类通用对话模型，temperature 0.2，要求输出 JSON schema；只能产生建议或命令请求，命令请求必经白名单+策略校验 |
| | fast path 实际条件与阈值 | moisture<20 且新鲜且 auto → pump_on(max_s=60)；moisture>85 → 告警不浇水；传感失效/无新数据 → 告警+skip |
| | 记忆/RAG 实际实现与是否启用 | state 有 memory 字段，启用“最近 N=5 条事件摘要”作为短期上下文；长期记忆/向量 RAG 未实现，仅有关键词降级路径（top-k=3，约 20 条护理知识文本） |
| | WebSocket 库、新鲜度阈值、队列大小 | websockets 库，端口 8765（HTTP 静态页 8000）；新鲜度阈值：传感 10 s、泵状态 10 s、视觉观测 2 s；事件队列 asyncio.Queue maxsize=100，满则丢最旧并计数 |
| 验证 | 实机验证到哪一级（V0–V5，见第 13 章）、真实踩过的 3 个故障 | 到达 V4（实机全链路：摄像头+泵+网页），未做 V5 长时间；最高证据 L4；一次现场试验观察到土壤读数随浇水从 41 升到 58（单次佐证，非统计）。3 个故障：(1) 继电器低触发极性导致上电即开；(2) letterbox 还原框整体偏移（pad 未减）；(3) TCP 粘包/半包导致 JSON 偶发解析失败 |

## 口径备忘（仅自查，面试按 CV 说）

1. **fast path**：CV 写 "extreme states"；笔记中的示例把 fast path 阈值写成与本地启动阈值同为 56。教材按 CV 的"极端状态"讲，用教学阈值 20；需核对实际条件。
2. **时间戳**：CV 说观测带时间戳；笔记称 Hi3861 传感 JSON 的 timestamp 当前为 0、RK 用设备系统时间。面试按"观测带采集时间 + 服务端接收时间"讲，不主动说传感时间戳为 0。
3. **预处理**：对话中以 letterbox 讲解，读源码印象为直接 resize；教材两种都讲并要求核对。
4. **输出张量**：笔记按 YOLOv8 形态 (1,C,A) 讲；CV 是 YOLOv5，教材按 v5 讲。
5. **协议**：长度头有"8 字节"与"4 字节"两种印象；服务端对 Hi3861 流用括号配对。教材不写死具体字节数。
6. **接口风险**：此前静态阅读发现服务端动作枚举（start/stop）与设备端命令（water_on/water_off 等）之间未见适配，且设备远程开关可能要求手动模式。这是需核对风险，非已证实故障；面试按 CV 讲闭环，被追问时诚实说明验证到哪一级。
7. **语音 off 与手动**：语音关闭可能恢复自动，随后低湿会重开；不要说"手动 off 永久有效"。
8. **去重**：`event_id:scope` 约 60 s、分析前标记，只防重复分析，不是 exactly-once。
9. **训练**：`onnx2rknn.py` 只证明部署转换，不证明训练或 mAP/FPS。
10. **记忆/RAG**：笔记只确认 state 有字段，不能声称已实现长期记忆。
11. **自查表取值**：自查表取值为教材统一设计取值，若与实机不同以实机为准并同步修改。
