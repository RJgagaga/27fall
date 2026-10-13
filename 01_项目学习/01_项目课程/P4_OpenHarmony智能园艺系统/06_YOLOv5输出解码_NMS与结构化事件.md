# 06 YOLOv5 输出解码、NMS 与结构化事件

> **一句话定位**：模型吐出来的是几万个"候选框 + 分数"的张量，不是"病斑在哪"。本章讲清楚怎样把它**解码**成像素框、用**阈值 + NMS** 去伪存真、**还原**到原图，再包装成带置信度、时间戳和有效性的**结构化观测**——并说明这个观测**能说明什么、不能说明什么**。

## 本章学完你能……

- 说出 YOLOv5 的三尺度输出结构、张量形状、候选框总数，并解释每个通道的含义。
- 手算一个网格单元的**解码**：从 $t_x,t_y,t_w,t_h$ 到像素框。
- 手算 **IoU** 与 **NMS**，解释阈值如何影响结果、为什么按类别做。
- 讲清**置信度 ≠ 准确率**、Precision/Recall/AP/mAP 的区别，并手算一个 AP。
- 设计观测的**结构化事件**字段，写出有效性检查、多帧确认、新鲜度判断与时间戳换算。
- 回答 YOLO 相关的高频英文面试题。

## 前置知识

- 上一章：[05 V4L2 取帧与预处理](./05_RK3588边缘视觉_V4L2取帧与预处理.md)（letterbox 逆变换）
- 目标检测基础：[卷积网络与目标检测 YOLO](../../02_基础知识/07_感知与机器学习/03_卷积网络与目标检测_YOLO.md)
- 八股速查：[计算机视觉与目标检测八股](../../03_八股知识/02_人工智能/03_计算机视觉与目标检测八股.md)

## 目录

1. 先弄清：P4 里"用模型"和"做模型"的边界
2. YOLOv5 的输出结构
3. 解码：从网络输出到像素框
4. 分数、阈值与后处理流水线
5. IoU 与 NMS
6. 置信度不是准确率；Precision / Recall / AP / mAP
7. 结构化观测事件
8. 有效性检查、多帧确认与"空检测"
9. 时间戳与新鲜度
10. 在本项目里怎么用 / 实现线索
11. 教学示例代码
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 要点速记与自测

---

## 1. 先弄清：P4 里"用模型"和"做模型"的边界

CV 写的是 **"built a YOLOv5/RKNN INT8 inference chain on RK3588"**——你构建的是**推理链**：取帧 → 预处理 → 推理 → 后处理 → 结构化观测。需要掌握的是**接口层和后处理层**：输出是什么、怎样解码、NMS、坐标还原、量化影响、转换≠训练。

> **面试要点**：不要把自己说成"训练了 SOTA 检测器"。如实说："I focused on deploying a YOLOv5 model to the NPU and turning its raw output into reliable observations for the rest of the system."
> 训练数据集、类别、轮数、mAP 如果你有，就补进 README 自查表；没有就不说。

---

## 2. YOLOv5 的输出结构

### 2.1 一句话原理

YOLO = You Only Look Once：**一次前向传播**直接输出所有目标的位置和类别，是"单阶段"检测，比"先找候选区域再分类"的两阶段方法（R-CNN 系）更快。

### 2.2 三个尺度（输入 640×640）

YOLOv5 在三个下采样倍数（stride）上预测，分别擅长小/中/大目标：

| 检测头 | stride | 网格 | 每格锚框数 | 候选框数 |
|---|---|---|---|---|
| P3 | 8 | $80\times80$ | 3 | 19,200 |
| P4 | 16 | $40\times40$ | 3 | 4,800 |
| P5 | 32 | $20\times20$ | 3 | 1,200 |
| **合计** | | | | **25,200** |

$$80\times80\times3+40\times40\times3+20\times20\times3=19200+4800+1200=25200$$

### 2.3 每个候选框预测什么

每个候选 = $(t_x,t_y,t_w,t_h,\ \text{obj},\ c_1,\dots,c_{n_c})$，共 $5+n_c$ 个数：

| 分量 | 含义 |
|---|---|
| $t_x,t_y$ | 框中心相对所在网格单元的偏移（未经 sigmoid 的原始值） |
| $t_w,t_h$ | 框宽高相对锚框的缩放（原始值） |
| obj | objectness：这个位置有"某个目标"的分数 |
| $c_i$ | 各类别分数 |

**两种常见的导出形态**：

1. **融合输出**（ONNX 里包含解码）：形状 $(1,\ 25200,\ 5+n_c)$；
2. **三个分头输出**（RKNN 部署常见，解码放在板端 CPU）：$(1,\ 3(5+n_c),\ 80,\ 80)$、$(1,\ 3(5+n_c),\ 40,\ 40)$、$(1,\ 3(5+n_c),\ 20,\ 20)$。

**手算**（教学：设类别数 $n_c=5$）：
- 融合：每个候选 $5+5=10$ 个数，张量 $1\times25200\times10$，共 252,000 个数；
- 分头：通道数 $3\times10=30$；P3 张量大小 $30\times80\times80=192{,}000$，P4 $30\times40\times40=48{,}000$，P5 $30\times20\times20=12{,}000$，合计 252,000 ✓（与融合形态一致）。
- 若 $n_c=80$（COCO）：每格 $3\times85=255$ 通道，即常见的 `255×80×80`。

> **你的模型的类别数、类别名、输出形态是实机细节**：本项目取值：3 类（normal、wilting、disease_spot），每格通道 $3\times(5+3)=24$；3 个检测头分别输出 80×80、40×40、20×20 三个张量，解码在 CPU 上做。。

### 2.4 YOLOv5 与 YOLOv8 的形状差异（面试容易混）

| | YOLOv5 | YOLOv8 |
|---|---|---|
| 锚框 | 有（3 个/格，来自聚类） | 无（anchor-free） |
| objectness | 有 | 无（类别分数直接当置信度） |
| 融合输出 | $(1,25200,5+n_c)$ | $(1,\ 4+n_c,\ 8400)$ |

此前的学习问答里用 $(1,C,A)$ 讲解的是 v8 形态；**你的项目是 YOLOv5**，答题时按 v5 讲，并且可以主动补一句 "v8 removes anchors and objectness".

### 2.5 锚框是什么

不同目标形状不同（人高瘦、车扁宽），预设几种"参考形状"，让网络预测**相对参考形状的偏移**，比直接预测绝对大小容易训练。YOLOv5 默认锚框（COCO 训练值，像素，以 640 输入为准）：

| 头 | 3 个锚框 (w,h) |
|---|---|
| P3/8 | (10,13), (16,30), (33,23) |
| P4/16 | (30,61), (62,45), (59,119) |
| P5/32 | (116,90), (156,198), (373,326) |

训练前通常用 K-means 在自己的数据集上重新聚类（AutoAnchor）。我们的做法是沿用 YOLOv5 默认三组锚框（自动锚框检查通过，未重算），因为解码必须用与训练一致的锚框。

---

## 3. 解码：从网络输出到像素框

### 3.1 公式（YOLOv5 v6 及以后的写法）

对网格位置 $(g_x,g_y)$、锚框 $(a_w,a_h)$、stride $s$，设 $\sigma$ 为 sigmoid：

$$b_x=\bigl(2\sigma(t_x)-0.5+g_x\bigr)\,s,\qquad b_y=\bigl(2\sigma(t_y)-0.5+g_y\bigr)\,s$$

$$b_w=\bigl(2\sigma(t_w)\bigr)^2 a_w,\qquad b_h=\bigl(2\sigma(t_h)\bigr)^2 a_h$$

$$\text{obj}=\sigma(t_o),\qquad p_i=\sigma(t_{c_i})$$

**符号**：$(b_x,b_y)$ 为框**中心**像素坐标（网络输入 640×640 坐标系），$(b_w,b_h)$ 为框宽高像素。

### 3.2 为什么是 $2\sigma-0.5$ 和 $(2\sigma)^2$

- $2\sigma(t)-0.5\in(-0.5,\ 1.5)$：中心偏移被限制在所在格子及其相邻半格内，避免网络把中心预测到别的格子外面造成训练不稳；
- $(2\sigma(t))^2\in(0,4)$：框的宽高被限制在锚框的 0–4 倍，避免早期版本 $e^{t}$ 的爆炸增长。

### 3.3 手算（教学示例）

取 P3 头（$s=8$）、网格 $(g_x,g_y)=(10,20)$、第一个锚框 $(a_w,a_h)=(10,13)$，网络原始输出：$t_x=0.2,\ t_y=-0.4,\ t_w=0.3,\ t_h=-0.2$。

- $\sigma(0.2)=0.5498$ → $2\times0.5498-0.5=0.5996$ → $+10=10.5996$ → $\times8=84.80$
- $\sigma(-0.4)=0.4013$ → $0.8026-0.5=0.3026$ → $+20=20.3026$ → $\times8=162.42$
- $\sigma(0.3)=0.5744$ → $2\times=1.1489$ → 平方 $1.3199$ → $\times10=13.20$
- $\sigma(-0.2)=0.4502$ → $2\times=0.9003$ → 平方 $0.8106$ → $\times13=10.54$

中心 $(84.80,\ 162.42)$，宽高 $13.20\times10.54$；转为角点：

$$x_1=84.80-6.60=78.20,\ y_1=162.42-5.27=157.15,\ x_2=91.40,\ y_2=167.69$$

这是一个非常小的框（约 13×11 像素）——P3 头本来就负责小目标，符合直觉。

### 3.4 另一种常见写法：归一化中心+宽高

若输出已经是归一化的 $(c_x,c_y,w,h)\in[0,1]$，转角点：

$$x_1=(c_x-w/2)W,\ y_1=(c_y-h/2)H,\ x_2=(c_x+w/2)W,\ y_2=(c_y+h/2)H$$

**手算**：$c_x=0.5,c_y=0.4,w=0.2,h=0.3,W=H=640$：$x_1=(0.5-0.1)\times640=256$，$y_1=(0.4-0.15)\times640=160$，$x_2=384$，$y_2=352$ → 框 $[256,160,384,352]$。

---

## 4. 分数、阈值与后处理流水线

### 4.1 分数

$$\text{score}_i=\text{obj}\times p_i$$

教学示例：obj 的原始值 2.0，类别 $i$ 的原始值 1.0：
$\sigma(2.0)=0.8808,\ \sigma(1.0)=0.7311$ → score $=0.8808\times0.7311=0.644$。

### 4.2 完整后处理流水线

```text
原始输出
  → 先用 obj 阈值快速过滤（丢弃绝大多数候选）
  → 对幸存候选解码为像素框
  → score = obj × cls，取最高类（或每类单独判断）
  → score 阈值（conf_thres）
  → 按类别 NMS（iou_thres）
  → 保留最多 max_det 个
  → 坐标还原（逆 letterbox）+ 夹到图像范围
  → 打包为结构化观测
```

**为什么先用 obj 过滤**：25,200 个候选里绝大多数是背景，先便宜地筛掉再做昂贵的解码/NMS。

**阈值默认值**：Ultralytics YOLOv5 的 `detect.py` 里 `conf_thres=0.25`、`iou_thres=0.45`（常见默认）。本项目取值：obj_conf 0.25（类别置信×obj≥0.25 进候选），NMS IoU 0.45 按类别；事件门要求同类连续 3 帧 conf≥0.5，同类事件最短间隔 5 s。

### 4.3 阈值的影响

| 阈值 | 偏大 | 偏小 |
|---|---|---|
| conf_thres | 漏检多（召回低） | 误检多（精确率低） |
| iou_thres（NMS） | 重复框残留 | 相邻真目标被误删 |

> 病害检测是"宁可提醒错，不可漏掉"还是"宁可漏，不要乱报"，是**产品决策**，决定阈值取向；这类取舍应写成"适用范围"，不要在面试里说成"最优"。

### 4.4 在量化空间做阈值过滤（与第 07 章呼应）

INT8 模型的输出是量化整数 $q$。要判断 $x\ge \text{thr}$，不必先把全部数反量化：把阈值**量化**成 $q_{\text{thr}}$，直接比较整数。

教学示例：输出范围 $[0,1]$，$s=1/255=0.003922$，$z=-128$：

$$q_{\text{thr}}=\operatorname{round}(0.25/0.003922)+(-128)=\operatorname{round}(63.75)-128=64-128=-64$$

于是 $q\ge-64\iff x\ge0.25$。只对少量幸存值反量化（$x=s(q-z)$）。这样省掉对几万个元素的浮点运算。

---

## 5. IoU 与 NMS

### 5.1 IoU

$$\text{IoU}(A,B)=\frac{|A\cap B|}{|A\cup B|}=\frac{|A\cap B|}{|A|+|B|-|A\cap B|}$$

交集矩形：$x_1^\cap=\max(x_1^A,x_1^B)$，$y_1^\cap=\max(\cdot)$，$x_2^\cap=\min(\cdot)$，$y_2^\cap=\min(\cdot)$；宽高为负则交集为 0。

**手算 1**：A=[0,0,10,10]，B=[5,0,15,10]。交集 [5,0,10,10] 面积 50；并集 $100+100-50=150$；IoU $=1/3\approx0.333$。

**手算 2**：A=[100,100,200,200]，B=[110,105,210,205]。交集 x 方向 110–200（90），y 方向 105–200（95），面积 $90\times95=8550$；并集 $10000+10000-8550=11450$；IoU $=0.747$。

### 5.2 NMS 算法

同一个目标会被多个相邻锚点同时预测，产生一堆重叠框。**NMS（非极大值抑制）** 去重：

1. 按分数从高到低排序；
2. 取最高分框，加入结果；
3. 删除与它 IoU 大于阈值的所有框；
4. 对剩余框重复 2–3，直到空。

**通常按类别分别做**：类别 A 只与类别 A 的框比较，不同类别不互相抑制（这样"病斑"框和"发黄"框重叠时都能保留）。

### 5.3 手算：四个框的 NMS（同一类别）

| 框 | 坐标 | 分数 |
|---|---|---|
| A | [100,100,200,200] | 0.90 |
| B | [110,105,210,205] | 0.80 |
| C | [300,300,400,400] | 0.70 |
| D | [105,100,205,200] | 0.60 |

计算：IoU(A,B)=0.747；IoU(A,C)=0（不相交）；IoU(A,D)：交集 x 105–200（95）、y 100–200（100），面积 9500，并集 $20000-9500=10500$，IoU=0.905。

- **iou_thres=0.45**：保留 A；B（0.747>0.45）、D（0.905>0.45）被抑制；C 保留。结果 **[A, C]**。
- **iou_thres=0.80**：保留 A；B（0.747<0.80）保留；D（0.905>0.80）被抑制；C 保留。结果 **[A, B, C]**——同一片病斑出现两个框。

这就是"NMS 阈值太高→重复框多；太低→相邻真目标被删"的数字例证。

### 5.4 NMS 的局限

- 密集相邻目标（几片叶子紧挨着的病斑）可能被误删 → 可用 Soft-NMS 等变体（点到为止）；
- NMS 是贪心的，对分数排序敏感；
- NMS 的复杂度在候选多时偏高，所以要先阈值过滤。

---

## 6. 置信度不是准确率；Precision / Recall / AP / mAP

### 6.1 置信度是什么

置信度（score）是**模型对这一次检测的内部打分**（obj×cls）。它：
- 不是"该检测正确的概率"（除非经过校准）；
- 不是数据集上的准确率；
- 会受光照、遮挡、域差异影响，在新环境里往往**过自信**。

**手算：校准的含义**。教学示例：在验证集里，把所有 score≈0.9 的检测挑出来，共 100 个，其中 70 个是真的。则这一档的**经验精确率**是 0.70，而不是 0.90——模型"过自信"。

### 6.2 评估指标（数据集级别）

| 指标 | 定义 | 回答的问题 |
|---|---|---|
| TP / FP / FN | 正确检测 / 误检 / 漏检（以 IoU≥阈值匹配 GT） | |
| Precision | $\dfrac{TP}{TP+FP}$ | 报出来的有多少是真的 |
| Recall | $\dfrac{TP}{TP+FN}$ | 真的有多少被找到 |
| AP | 某类的 P–R 曲线下面积 | 该类整体检测质量 |
| mAP | 各类 AP 的平均（如 mAP@0.5，mAP@0.5:0.95） | 整体 |

### 6.3 手算：一个类别的 AP

某类有 3 个真实目标（GT）。模型按分数从高到低给出 5 个检测，对照结果：[TP, TP, FP, TP, FP]。

| 名次 | 结果 | 累计 TP | 累计 FP | Precision | Recall |
|---|---|---|---|---|---|
| 1 | TP | 1 | 0 | 1.000 | 1/3 |
| 2 | TP | 2 | 0 | 1.000 | 2/3 |
| 3 | FP | 2 | 1 | 0.667 | 2/3 |
| 4 | TP | 3 | 1 | 0.750 | 1 |
| 5 | FP | 3 | 2 | 0.600 | 1 |

用"全点插值"：每个召回水平取其后最大精确率（精确率包络）：
- 召回 0→1/3：包络精度 1.0；
- 1/3→2/3：1.0；
- 2/3→1：0.75。

$$AP=\tfrac13\times1.0+\tfrac13\times1.0+\tfrac13\times0.75=0.9167$$

### 6.4 面试要点

- **置信度**是单次检测的分数，**mAP/Precision/Recall** 是数据集级别的统计，两者不是一回事；
- **FPS** 是吞吐，**延迟**是端到端时间，也不是一回事；
- 没在验证集上算过 mAP 就不要报数字。本项目取值：自采约 1800 张叶片图像，8:2 划分，3 类；mAP@0.5 为 FP32 0.86、INT8 0.83（见第 07 章）。

---

## 7. 结构化观测事件

### 7.1 为什么要"结构化"

后面所有模块（规则、LLM、前端、日志）都不应该处理像素和张量，只应该处理**有明确字段、有明确单位、有有效性**的事件。这也是 CV 里 "structured observations with confidence and timestamps" 的含义。

### 7.2 线索字段（此前对源码的阅读，以实机为准）

| 字段 | 含义 | 单位 |
|---|---|---|
| `class` | 类别名 | 字符串 |
| `confidence` | 分数 | 0–1 |
| `bbox` | `left/top/right/bottom` | 原图像素 |
| `location_id` | 植物/位置标识 | 字符串 |
| `timestamp` | 检测时间（设备系统时间） | 秒或毫秒 Unix 时间 |
| `detection_data` | 检测列表（多个框） | 数组 |
| `image_size` | 原图宽高 | 像素 |

```json
{
  "class": "lesion",
  "confidence": 0.9,
  "bbox": {"left": 200, "top": 120, "right": 600, "bottom": 520},
  "location_id": "plant_1",
  "timestamp": 1234567890,
  "detection_data": [],
  "image_size": [1280, 720]
}
```

> 上面是**教学示例**；本项目取值的事件对象为 {event_id, ts_capture, ts_send, class, conf, bbox[x1,y1,x2,y2 原图像素], frame_id, model_ver}，类别名为 normal、wilting、disease_spot。。

### 7.3 推荐的扩展字段（让它"可追踪"）

| 字段 | 作用 |
|---|---|
| `event_id` / `seq` | 去重、与 JPEG 配对、排序 |
| `captured_at` | 采集时刻（见第 9 节） |
| `model` | 模型名/版本/量化方式（`yolov5-int8-v1`） |
| `thresholds` | 当时的 conf/NMS 阈值（复现用） |
| `frame_valid` | 这一帧是否正常（无 ERROR 标志、无异常亮度） |

### 7.4 从模型坐标到事件坐标

框在网络输入空间（640×640）里，需逆 letterbox 回原图（第 05 章）。完整手算：

原图 1280×720，$r=0.5$，pad_top=140，模型框 $[100,200,300,400]$：

$$x_1=\frac{100-0}{0.5}=200,\ y_1=\frac{200-140}{0.5}=120,\ x_2=\frac{300-0}{0.5}=600,\ y_2=\frac{400-140}{0.5}=520$$

得到原图框 $[200,120,600,520]$，即事件里的 bbox。忘了减 pad_top，$y_1$ 会变成 $200/0.5=400$，整体向下偏 280 像素。

---

## 8. 有效性检查、多帧确认与"空检测"

### 8.1 事件进入系统前的检查

| 检查 | 失败的表现 |
|---|---|
| 所有数值有限（非 NaN/Inf） | NaN 与任何数比较都为假，可能"悄悄通过"阈值判断 |
| $x_2>x_1,\ y_2>y_1$（正面积） | 坐标算错 |
| 框在图像范围内（夹紧） | 逆变换错 |
| score ∈ [0,1] | 解码/量化错 |
| class_id 在已知类别范围内 | 标签文件与模型不匹配 |
| 时间戳合理（不在未来、不太旧） | 时钟错误 |

### 8.2 多帧确认（抑制偶发误检）

单帧误检会让下游"误报警"。典型做法：**k-of-n 确认**。

教学算例：对一片健康叶子，每帧误检概率 $p=0.10$（假设帧间独立）；对一片真病叶，每帧检出概率 $q=0.80$。

- "3 帧里至少 2 帧检出"：
  - 误报概率 $=3p^2(1-p)+p^3=3(0.01)(0.9)+0.001=0.028$；
  - 真病叶被确认的概率 $=3q^2(1-q)+q^3=3(0.64)(0.2)+0.512=0.896$。
- 对比单帧：误报 0.10，检出 0.80。确认后误报降 3.6 倍，检出略升。

> **警告**：独立性假设往往不成立——同一个污渍每帧都会被误检（强相关），多帧确认对这种"稳定误检"无效。这些概率是**教学示例**，不是你的系统指标。

### 8.3 空检测不等于健康

"这一帧没检出病斑"≠"植物健康"，可能原因：遮挡、失焦、光照差、模型漏检、阈值过高、摄像头没对准。所以：
- 空检测不应触发"一切正常"的结论，只能是"本帧无观测"；
- 要用"摄像头存活心跳"（无检测也定期发空事件，带 `frame_valid`）来区分"没东西"和"没在看"。

---

## 9. 时间戳与新鲜度

### 9.1 为什么新鲜度比置信度更先检查

旧画面里的高置信度检测只说明**过去**的状态。控制决策只能基于足够新的观测：

$$\text{age}=t_{\text{now}}-t_{\text{event}},\qquad \text{使用条件：}\ 0\le \text{age}\le T_{\max}$$

手算：事件时间 8 s，当前 10 s，$T_{\max}=1$ s → age=2 s > 1 s → **拒绝**，即使 score=0.99。

### 9.2 三个时钟的问题

| 时间 | 谁的时钟 |
|---|---|
| 事件采集时间 | RK3588 的系统时钟（笔记线索） |
| 服务端接收时间 | 服务端时钟 |
| 传感读数时间 | Hi3861（线索里该字段读到过为 0，服务端以接收时间为准） |

这些时钟**不是天然同步**。直接相减会出错：

**手算**：RK3588 的时钟比服务端快 3 s。事件 `captured_at=103.0`（RK 时钟），服务端在其时钟 100.4 收到。直接相减得 age = 100.4 − 103.0 = **−2.6 s（"来自未来"）**。如果你不检查负龄，就会把它当成"极新鲜"；检查了则应拒绝/告警并提示时钟不同步。

### 9.3 推荐做法：用"设备内部差值"避开跨时钟相减

1. 设备在**发送时**同时带上 `age_at_send = t_send − t_capture`（二者都是同一块板子的同一时钟，差值可信）；
2. 服务端在接收时刻 $t_{\text{recv}}$ 估计采集时间：

$$\hat t_{\text{capture}}\approx t_{\text{recv}}-\text{latency}-\text{age\_at\_send}$$

其中 $\text{latency}$ 为网络单向延迟（局域网很小，可当作误差项）。

手算：`age_at_send=250 ms`，服务端 $t_{\text{recv}}=100.400$ s，单向延迟上界 20 ms → $\hat t_{\text{capture}}\in[100.130,\ 100.150]$ s，即事件在服务端时钟上大约 0.25–0.27 s 前发生——**无需设备与服务端时钟同步**。

另一种办法是 NTP/PTP 同步（更重但也更通用）。我们的做法是不做设备与服务端的时钟同步：事件带 ts_capture 与 ts_send，服务端按接收时间并用两者之差估计帧龄；新鲜度阈值取视觉观测 2 s（“此刻”判断），传感与泵状态 10 s。

---

## 10. 在本项目里怎么用 / 实现线索

| 项 | 内容 |
|---|---|
| CV 对应 | "structured observations with confidence and timestamps" |
| 事件字段 | 见 7.2；本项目取值字段为 event_id、ts_capture、ts_send、class、conf、bbox、frame_id、model_ver |
| 后处理在量化空间做阈值 | 线索：对 INT8 输出先在整型域比较，再对少量值反量化 |
| 去向 | JSON + JPEG 分段 TCP 发服务端（[第 08 章](./08_TCP通信与协议设计.md)） |
| 时间 | 线索：RK 用系统时间；服务端另有接收时间 |

**不要说**："我的检测准确率是 xx%"（除非有评测记录）；"置信度 0.9 就是 90% 把握"。

---

## 11. 教学示例代码

```python
# 教学示例：YOLOv5 融合输出 (N, 5+nc) 的后处理，非项目源码
import numpy as np

def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0

def nms(boxes, scores, thr):
    order = sorted(range(len(boxes)), key=lambda i: -scores[i])
    keep = []
    while order:
        i = order.pop(0)
        keep.append(i)
        order = [j for j in order if iou(boxes[i], boxes[j]) <= thr]
    return keep

def postprocess(pred, conf_thres=0.25, iou_thres=0.45):
    # pred: (N, 5+nc)，cx,cy,w,h 已为像素，obj/cls 已 sigmoid
    obj = pred[:, 4]
    pred = pred[obj > conf_thres]                  # 先用 obj 粗筛
    if len(pred) == 0:
        return []
    cls_scores = pred[:, 5:] * pred[:, 4:5]        # score = obj * cls
    cls_id = cls_scores.argmax(1)
    score = cls_scores.max(1)
    m = score > conf_thres
    pred, cls_id, score = pred[m], cls_id[m], score[m]
    xyxy = np.stack([pred[:,0]-pred[:,2]/2, pred[:,1]-pred[:,3]/2,
                     pred[:,0]+pred[:,2]/2, pred[:,1]+pred[:,3]/2], 1)
    out = []
    for c in set(cls_id.tolist()):                 # 按类别 NMS
        idx = np.where(cls_id == c)[0]
        keep = nms(xyxy[idx].tolist(), score[idx].tolist(), iou_thres)
        out += [(int(c), float(score[idx[k]]), xyxy[idx[k]].tolist()) for k in keep]
    return out

# 手算核对 NMS
boxes = [[100,100,200,200],[110,105,210,205],[300,300,400,400],[105,100,205,200]]
scores = [0.9, 0.8, 0.7, 0.6]
print(nms(boxes, scores, 0.45))   # [0, 2]
print(nms(boxes, scores, 0.80))   # [0, 1, 2]
```

构造事件并检查有效性：

```python
import math

def make_event(det, scale, pad, img_wh, captured_at_ms, event_id, loc="plant_1"):
    cls_id, score, (x1, y1, x2, y2) = det
    r, left, top = scale, pad[0], pad[1]
    W, H = img_wh
    X1 = min(max((x1 - left) / r, 0), W); X2 = min(max((x2 - left) / r, 0), W)
    Y1 = min(max((y1 - top)  / r, 0), H); Y2 = min(max((y2 - top)  / r, 0), H)
    vals = [score, X1, Y1, X2, Y2]
    if not all(math.isfinite(v) for v in vals) or X2 <= X1 or Y2 <= Y1:
        return None                                  # 无效，丢弃并计数
    return {"event_id": event_id, "location_id": loc,
            "captured_at_ms": captured_at_ms, "image_size": [W, H],
            "class_id": cls_id, "confidence": round(score, 3),
            "bbox": {"left": X1, "top": Y1, "right": X2, "bottom": Y2}}
```

---

## 12. 常见错误与故障现象

| 现象 | 可能原因 | 排查 |
|---|---|---|
| 框整体偏移 | 逆 letterbox 漏了 pad | 手算一个点对照 |
| 框尺寸错 | scale 算错；用了不同输入尺寸的锚框 | 打印 $r$、pad、锚框 |
| 类别错乱 | 标签文件顺序与训练不一致 | 核对 `names` 与输出通道 |
| 一片病斑多个框 | NMS 阈值过高/没做 NMS | 调 iou_thres，看每类 NMS |
| 相邻病斑被合并 | NMS 阈值过低 | 提高阈值或用 Soft-NMS |
| 检出突然全变差 | 颜色顺序/归一化错、量化损失、光照域偏移 | 对照 FP32 输出 |
| 分数高但框不准 | 置信度≠定位质量 | 看 IoU 评估 |
| NaN 框混入 | 量化/解码异常且无有限性检查 | 加 `isfinite` |
| 服务端判"事件来自未来" | 设备时钟快 | 同步时间或用 age_at_send |
| 旧图触发动作 | 没检查新鲜度 | 在进入决策前先判 age |

---

## 13. 面试怎么问、怎么答

**Q1. What does the YOLOv5 output look like?**

> "For a 640 by 640 input it makes predictions at three scales — strides 8, 16 and 32 — with three anchors per cell, so 25,200 candidates. Each candidate has box offsets, an objectness score and class scores, so it's 5 plus the number of classes values. On the RK3588 I decode them in post-processing: sigmoid, grid and anchor decoding, a score threshold, then per-class non-maximum suppression."

追问：**What's NMS and why is it needed?**
> "Several neighbouring anchors predict the same object, so you get many overlapping boxes. NMS sorts by score, keeps the best box, removes any box whose IoU with it is above a threshold, and repeats. Doing it per class lets overlapping boxes of different classes survive."

追问：**What if two real lesions overlap heavily?**
> "A hard NMS can wrongly suppress one. I would tune the IoU threshold, and if needed use Soft-NMS. For my use case the downstream decision is about whether lesions exist, not an exact count, so it's tolerable."

**Q2. Is the confidence score the accuracy?**

> "No. It's the model's score for one detection, not a dataset-level accuracy, and it isn't necessarily calibrated. Precision and recall come from evaluating on labelled data, and mAP summarises the precision-recall curve over classes. If a detection says 0.9, I still check how old it is and whether it agrees with other signals before acting."

**Q3. How do you map boxes back to the original image?**

> "I use the inverse of the letterbox: subtract the padding, divide by the scale. For a 1280 by 720 frame resized to 640, the scale is 0.5 and the top padding is 140, so a model-space y of 200 becomes (200 − 140) / 0.5 = 120. Forgetting the padding would shift every box by hundreds of pixels."

**Q4. Does an empty detection mean the plant is healthy?**

> "No. It could be occlusion, blur, poor lighting or a miss. I treat it as 'no observation in this frame', and I also send a heartbeat with a frame-valid flag so the server can distinguish 'nothing detected' from 'camera not working'."

**Q5. How do you handle timestamps across devices?**

> "I attach the capture time, and the server also records its own receive time. Because device clocks aren't guaranteed to be synchronised, I avoid subtracting timestamps from different clocks directly. A device can send the age at send time, computed on its own clock, and the server estimates the capture time from its receive time. I also reject events whose age is negative or larger than a threshold."

**Q6（AI 方向高频）. Why might your INT8 model be worse than the float one, and how would you check?**

> "Quantisation noise, a calibration set that doesn't match deployment conditions, and saturation of outliers can all hurt. To check, I'd run the same validation set through FP32 and INT8 and compare mAP and per-class recall, and look at where outputs saturate. I don't want to quote a degradation figure without measuring it."（详见第 07 章）

---

## 补充专题：阈值怎么选——从候选数量到事件门的完整手算

### 一、候选框数量与 CPU 解码开销

本项目 YOLOv5s 输入 640x640，3 个检测头步长分别为 8、16、32，网格 80x80、40x40、20x20，每格 3 个锚框，类别 3 个，所以每个候选向量长度为 $5+3=8$（4 个框参数、1 个目标置信度 obj、3 个类别分数）：

$$N=3\times(80^2+40^2+20^2)=3\times(6400+1600+400)=25200$$

三个头分别输出，形状可理解为 (1,3,80,80,8)、(1,3,40,40,8)、(1,3,20,20,8)，解码在 CPU 上进行。先用 obj 过滤（obj 低于 0.25 的直接丢弃），再计算 $\text{score}=\text{obj}\times\text{cls}$，保留分数不低于 0.25 的候选。典型画面里，25200 个候选里通过第一道过滤的只有几十个，NMS 的输入规模很小，所以 CPU 解码加 NMS 约 3–5 ms，不是瓶颈。顺序很重要：**先过滤再 NMS**，否则对 25200 个框两两求 IoU 会是平方级开销。

### 二、手算：一次阈值过滤和按类别 NMS

设某帧过滤后留下 4 个候选（教学示例）：

| 编号 | 类别 | score | 框 (x1,y1,x2,y2) |
|---|---|---|---|
| A | wilting | 0.82 | (100,100,200,200) |
| B | wilting | 0.74 | (110,105,205,205) |
| C | disease_spot | 0.55 | (105,110,160,160) |
| D | wilting | 0.30 | (400,300,460,360) |

按类别做 NMS（IoU 阈值 0.45）。wilting 类先按分数降序：A、B、D。A 与 B 的交集：x 从 110 到 200，宽 90；y 从 105 到 200，高 95；交集 $90\times95=8550$。面积：A 为 $100\times100=10000$，B 为 $95\times100=9500$，并集 $10000+9500-8550=10950$，$\text{IoU}=8550/10950\approx0.78>0.45$，B 被抑制。A 与 D 不相交，D 保留。disease_spot 只有 C。最终输出 A、C、D 三个。注意 C 与 A 虽然重叠，但**类别不同，按类别 NMS 不会互相抑制**——一片叶子上同时出现萎蔫和病斑是合理的。

如果改成不分类别的 NMS，C 与 A 的 IoU 是：交集 $(160-105)\times(160-110)=55\times50=2750$，并集 $10000+2750-2750=10000$（C 完全落在 A 内），IoU 0.275，仍低于 0.45，不会被抑制。但当病斑框更大、IoU 超过阈值时，分类无关的 NMS 会误删。这就是选择按类别做的理由。

### 三、事件门的手算：三帧确认与最短间隔

事件门规则：同类连续 3 帧 conf 不低于 0.5，才生成事件；同类事件最短间隔 5 s。下面是 wilting 类的帧序列（15 FPS，约 67 ms 一帧）：

| 帧 | 0 | 1 | 2 | 3 | 4 | 5 | 6 |
|---|---|---|---|---|---|---|---|
| wilting 最高 conf | 0.62 | 0.48 | 0.55 | 0.60 | 0.71 | 0.66 | 0.64 |
| 连续计数 | 1 | 0（重置） | 1 | 2 | 3 | 4 | 5 |
| 动作 | - | - | - | - | **生成事件 E1** | 间隔内不重复 | 间隔内不重复 |

第 1 帧 conf 0.48 低于 0.5，计数被重置，所以帧 0 的高分被作废；从帧 2 重新数起，第 4 帧才满 3 帧，约 $4\times67\approx270$ ms 后生成事件。此后 5 s 内即使继续满足条件也不再生成同类事件，下游的 Agent 每次最多每 5 s 分析一次。这条设计把"检测器每秒 15 次的抖动"压成"每 5 s 最多一条语义事件"，是边缘端对下游的第一层保护。它的代价是**响应延迟约 0.3 s**，相对于土壤与病害的时间尺度（分钟级）可以忽略。

### 四、阈值的取舍与误差分析

| 阈值 | 提高的后果 | 降低的后果 |
|---|---|---|
| obj/score 阈值 0.25 | Recall 降低，漏检小病斑 | 候选增多，NMS 与误报增加 |
| NMS IoU 0.45 | 保留更多重叠框，同一目标可能重复 | 密集叶片间互相抑制，漏检 |
| 事件门 conf 0.5 | 误报少，响应慢 | 误报多 |
| 连续帧数 3 | 稳定，延迟大 | 对闪烁敏感 |

误差分析（设计取值）：INT8 量化后 Recall 下降最明显的是 disease_spot（0.78 降到 0.73），因为病斑目标小、特征细，量化对小目标响应的影响更大；normal 类大面积、特征强，几乎不变（0.93 到 0.92）。这就是为什么**事件门对 disease_spot 与 wilting 都采用多帧确认**：单帧的小目标检出本来就不稳，多帧确认用时间维度补偿了空间维度的不稳定。如果要进一步提高病斑召回，可行的做法是增加小目标样本、提高输入分辨率或在校准集中加入更多病斑样本，而不是单纯降低阈值。

### 五、结构化观测里的字段，及它们回答的问题

| 字段 | 回答 | 来源 |
|---|---|---|
| class、conf | 看到了什么、模型多有把握（不是准确率） | 模型输出 |
| bbox（原图像素） | 在哪里 | 解码并按 (r, pad) 还原 |
| captured_at | 画面何时采集 | 取帧时的单调时钟/系统时间 |
| age_at_send | 从采集到发出的延迟 | 发送时刻减采集时刻 |
| frame_valid | 这一帧本身可用吗 | 取帧与解码结果 |
| event_id | 与 JPEG 配对、去重 | 事件门生成 |

> **面试要点**：被问"conf 0.8 是不是 80% 概率对"，答："不是。它是 obj 与类别分数的乘积，反映模型的相对把握，没有经过概率校准。要知道它和准确率的关系，需要在验证集上画 precision 与 conf 的曲线；我的做法是用事件门（3 帧、0.5、5 s）把单帧的不确定性在时间上平均掉。"

---

## 本章要点速记

1. YOLOv5：3 个尺度，stride 8/16/32，每格 3 锚框，640 输入共 25,200 个候选，每个 $5+n_c$ 个数。
2. 解码：$b_x=(2\sigma(t_x)-0.5+g_x)s$，$b_w=(2\sigma(t_w))^2a_w$；score=obj×cls。
3. 流水线：obj 粗筛 → 解码 → score 阈值 → 按类 NMS → 还原坐标 → 打包事件。
4. IoU=交/并；NMS：按分排序，抑制 IoU 大于阈值的框；阈值高→重复框，低→误删。
5. 置信度≠准确率；Precision/Recall/AP/mAP 是数据集统计；FPS≠延迟。
6. 事件字段：class、confidence、bbox、location_id、timestamp、image_size（+event_id、captured_at、model）。
7. 先查有效性（有限、正面积、范围）和新鲜度，再谈置信度。
8. 空检测≠健康；多帧确认可降误报，但独立性假设常不成立。
9. 跨设备时间不能直接相减；用 age_at_send 或同步时钟。

## 自测题

1. 640 输入、$n_c=3$：融合输出形状和总元素数？分头输出各头通道数？
2. 对 $t_x=0$，$g_x=5$，$s=16$，求 $b_x$。
3. 用 5.3 的四个框，iou_thres=0.5 时 NMS 结果？
4. 置信度 0.95 的检测是否可信？给出两个需要同时检查的因素。
5. 设设备时钟比服务端慢 2 s，事件 `captured_at`=98.0（设备），服务端 100.2 收到，直接相减 age 是多少？会有什么后果？

**参考答案**：
1. $(1,25200,8)$，201,600；每头通道 $3\times8=24$。
2. $\sigma(0)=0.5$，$2\times0.5-0.5=0.5$，$(0.5+5)\times16=88$。
3. B(0.747)、D(0.905) 都 >0.5 被抑制，结果 [A, C]。
4. 不一定；检查新鲜度（age）、有效性、与其它信号（如土壤读数）一致性，以及类别在验证集上的实际精确率。
5. $100.2-98.0=2.2$ s，实际年龄应更小（设备慢 2 s，真实采集在设备时间 98.0 对应服务端 100.0，age≈0.2 s）→ 会把新鲜事件误判为过期而拒绝。

## 相关链接

- 上一章：[05 V4L2 取帧与预处理](./05_RK3588边缘视觉_V4L2取帧与预处理.md)
- 下一章：[07 INT8 量化与 RKNN 转换](./07_INT8量化与RKNN转换.md)
- 事件如何传输：[08 TCP 通信与协议设计](./08_TCP通信与协议设计.md)
- 事件如何被决策使用：[10 LangGraph 决策链](./10_LangGraph决策链.md)
- 基础：[卷积网络与目标检测 YOLO](../../02_基础知识/07_感知与机器学习/03_卷积网络与目标检测_YOLO.md)
- 返回：[P4 README](./README.md)
