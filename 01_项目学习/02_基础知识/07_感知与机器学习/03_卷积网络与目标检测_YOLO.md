# 卷积网络与目标检测 YOLO

> **一句话定位**：从"一次卷积怎么算"出发，一路讲到"YOLOv5 一张图怎么变成带置信度的框"，把你 P4 里的视觉模型从黑盒变成每一步都能手算、能英文解释的白盒。

**本章学完你能……**

- 手算一次二维卷积和池化，写出输出尺寸与参数量公式，算出任意堆叠的感受野。
- 讲清 LeNet→AlexNet→VGG→ResNet→MobileNet→CSP 各自解决什么问题。
- 手算 IoU、NMS、AP/mAP，并说出 mAP@0.5 与 mAP@0.5:0.95 的区别。
- 对比两阶段与单阶段检测，复述 YOLOv1→v5（含 v8 的差异）的演进主线。
- 画出 YOLOv5 的 Backbone / Neck(FPN+PAN) / Head 结构，说明 Focus、CSP(C3)、SPPF 的作用。
- 手算 YOLOv5 输出张量的解码（$2\sigma-0.5$、$(2\sigma)^2\times$ 锚框），知道 25200 这个数字怎么来的。
- 说清 YOLOv5 三项损失（CIoU 框回归、objectness、分类），数据增强的作用，评估时该看什么。
- 把这些知识接到边缘部署：为什么后处理、letterbox、量化会影响精度（接下一章）。

**前置知识**

- [02 神经网络与反向传播](./02_神经网络与反向传播.md)（前向、激活、BN）
- [01 机器学习基础](./01_机器学习基础.md)（precision/recall/AUC）

**目录**

1. 卷积网络基础
2. 感受野
3. 经典 CNN 一路演进
4. 目标检测的基本概念：框、IoU、锚框、NMS、AP
5. 两阶段与单阶段
6. YOLO 演进 v1→v5
7. YOLOv5 结构
8. 输出张量与解码
9. 损失函数
10. 数据增强
11. 评估
12. 边缘部署相关
13. 在你的项目里怎么用
14. 常见错误与故障现象
15. 面试问答
16. 自测题

---

## 1. 卷积网络基础

### 1.1 它要解决什么问题

一张 $640\times640\times3$ 的图展平有 122 万个数。全连接层接 1000 个神经元就有 12 亿参数，且完全不利用图像的结构。图像有两个性质：**局部相关**（相邻像素关系紧密）、**平移不变**（猫在左上角还是右下角都是猫）。卷积网络（CNN）把这两条写进结构：

- **局部连接**：每个输出只看一小块区域。
- **权重共享**：同一个小滤波器（kernel）滑过整张图。
- 结果：参数量与图像尺寸无关，对平移"等变"（输入平移，特征图跟着平移）。

### 1.2 符号

- 输入特征图 $X\in\mathbb R^{C_{in}\times H\times W}$；卷积核 $K\in\mathbb R^{C_{out}\times C_{in}\times k\times k}$；偏置 $b\in\mathbb R^{C_{out}}$。
- 步长（stride）$s$；填充（padding）$p$。
- 输出尺寸：

$$H_{out}=\Big\lfloor\frac{H+2p-k}{s}\Big\rfloor+1$$

- 单个输出值（深度学习里的"卷积"实际是互相关，不翻转核）：

$$Y_{o,i,j}=\sum_{c=1}^{C_{in}}\sum_{u=0}^{k-1}\sum_{v=0}^{k-1}K_{o,c,u,v}\,X_{c,\,is+u,\,js+v}+b_o$$

### 1.3 手算（教学示例数值）

输入 $5\times5$ 单通道，核为 $3\times3$ 的垂直边缘检测器，$s=1,p=0$，输出 $3\times3$（$\lfloor(5-3)/1\rfloor+1=3$）。

$$X=\begin{bmatrix}1&2&0&1&3\\0&1&2&3&1\\1&0&1&2&0\\2&1&0&1&2\\0&3&1&0&1\end{bmatrix},\quad K=\begin{bmatrix}1&0&-1\\1&0&-1\\1&0&-1\end{bmatrix}$$

核的含义：每行"左列减右列"，对左亮右暗的竖直边缘响应为正。

- 左上窗口 $\begin{bmatrix}1&2&0\\0&1&2\\1&0&1\end{bmatrix}$：$(1-0)+(0-2)+(1-1)=1-2+0=-1$。
- 窗口右移一格 $\begin{bmatrix}2&0&1\\1&2&3\\0&1&2\end{bmatrix}$：$(2-1)+(1-3)+(0-2)=1-2-2=-3$。
- 其余同理，得到
$$Y=\begin{bmatrix}-1&-3&-1\\0&-4&0\\1&1&-1\end{bmatrix}$$

### 1.4 参数量与计算量

卷积层参数 $=C_{out}(C_{in}k^2+1)$。例：$3\to16$ 通道、$3\times3$ 核：$16\times(3\times9+1)=448$ 个参数。若输出特征图 $320\times320$，乘加次数（MACs）$=3\times3\times3\times16\times320\times320\approx4.42\times10^7$。注意：**参数量与计算量不是一回事**：参数少但特征图大的层，计算量可以很高。

**对比全连接**（教学示例）：$640\times640\times3$ 展平为 $1{,}228{,}800$ 维，接 1000 个神经元的全连接层参数 $=1{,}228{,}800\times1000\approx1.23\times10^9$（12 亿）；而上面的卷积层只有 448 个参数，且换成 $1280\times1280$ 的输入参数量不变。这就是权重共享的威力；代价是每个输出点只"看见"局部，需要靠堆叠加深来扩大感受野。

**步长与填充的作用**：$s=2$ 使特征图边长减半，同时感受野增长加快；$p=\lfloor k/2\rfloor$（"same" 填充，$k$ 为奇数）在 $s=1$ 时保持尺寸不变，例如 $k=3,p=1$：$\lfloor(W+2-3)/1\rfloor+1=W$。填充用零还是反射镜像，会影响边界像素的响应。

### 1.5 其他常见层

- **池化（pooling）**：下采样，降分辨率、增大感受野、带来小幅平移不变。最大池化手算（$2\times2$，$s=2$）：
$$\begin{bmatrix}1&3&2&4\\5&6&1&2\\7&2&9&0\\3&4&8&1\end{bmatrix}\to\begin{bmatrix}6&4\\7&9\end{bmatrix}$$
现代网络更多用 $s=2$ 的卷积代替池化。
- **$1\times1$ 卷积**：只混合通道，不看空间邻域；用来升降维（瓶颈）、融合通道。
- **深度可分离卷积（depthwise separable）**：把标准卷积拆成"逐通道空间卷积 + $1\times1$ 通道混合"。$C_{in}=C_{out}=64,\ k=3$：标准卷积每个位置 $9\times64\times64=36864$ 次乘加；可分离 $9\times64+64\times64=576+4096=4672$，约为 $1/7.9$。这是 MobileNet 轻量化的核心。
- **空洞卷积**：核内插空，不增加参数地扩大感受野。
- **上采样**：最近邻/双线性或转置卷积，检测网络的 Neck 里用来放大特征图。
- **激活与 BN**：见上一章。现代卷积块的标准写法是 Conv → BN → 激活（YOLOv5 里叫 `Conv` 模块），部署时 BN 融合进 Conv。

---

## 2. 感受野

### 2.1 问题与定义

感受野（receptive field, RF）：某层一个输出点，对应输入图上多大的区域。检测大目标需要足够大的感受野，否则每个点只"看见"物体的一小块。

### 2.2 递推公式

设第 $l$ 层核大小 $k_l$、步长 $s_l$，定义"跳距" $j_l=\prod_{i\le l}s_i$（相邻输出点在输入上相隔多少像素），$j_0=1,\ r_0=1$：

$$r_l=r_{l-1}+(k_l-1)\,j_{l-1},\qquad j_l=j_{l-1}s_l$$

### 2.3 手算

四层：$(k,s)=(3,1),(3,1),(3,2),(3,1)$。

| 层 | $r$ 的计算 | $r_l$ | $j_l$ |
|---|---|---|---|
| 1 | $1+2\times1$ | 3 | 1 |
| 2 | $3+2\times1$ | 5 | 1 |
| 3 | $5+2\times1$ | 7 | 2 |
| 4 | $7+2\times2$ | 11 | 2 |

结论：两个 $3\times3$ 叠加的感受野是 $5\times5$，但参数更少（$2\times9C^2=18C^2$ 对 $25C^2$）且多一次非线性——这是 VGG 的核心设计理由。下采样之后再叠卷积，感受野增长更快（第 4 层增量是 4 而不是 2）。

---

## 3. 经典 CNN 一路演进

| 模型 | 年份 | 关键思想 | 解决的问题 |
|---|---|---|---|
| LeNet-5 | 1998 | 卷积+池化+全连接 | 手写数字，证明 CNN 可行 |
| AlexNet | 2012 | ReLU、Dropout、GPU、数据增强 | ImageNet 大幅领先，引爆深度学习 |
| VGG | 2014 | 全用 $3\times3$ 卷积堆深 | 小核堆叠等效大感受野、参数更省 |
| GoogLeNet/Inception | 2014 | 多分支并行 + $1\times1$ 降维 | 同层多尺度，控制计算量 |
| ResNet | 2015 | 残差连接 $y=x+F(x)$ | 深层网络退化/梯度消失，可到上百层 |
| MobileNet | 2017 | 深度可分离卷积 | 移动端/边缘设备轻量化 |
| CSPNet | 2019 | 跨阶段部分连接：特征分两路，一路过残差块，最后拼接 | 减少重复梯度信息、降计算量，YOLOv4/v5 的骨干基础 |

**残差连接为什么有效**（接上一章 10.2）：反向时 $\partial y/\partial x=1+\partial F/\partial x$，恒等项保证梯度至少有一条无损通路；同时网络可以轻松学成"恒等"，加深不会更差。

**瓶颈残差块手算**（ResNet-50 类结构，教学示例）：输入 256 通道。
- 直接用 $3\times3$ 卷积 $256\to256$：参数 $9\times256\times256=589{,}824$。
- 瓶颈结构：$1\times1$（$256\to64$，$16{,}384$）+ $3\times3$（$64\to64$，$36{,}864$）+ $1\times1$（$64\to256$，$16{,}384$）$=69{,}632$，约为前者的 $1/8.5$。先降维、在低维上做昂贵的空间卷积、再升维，参数和计算大幅下降，同时保留残差通路。YOLOv5 的 C3 里的 Bottleneck 沿用了这类"先 $1\times1$ 降通道再 $3\times3$"的思路。

**迁移学习**：在 ImageNet/COCO 上预训练的骨干，前几层学到的边缘、纹理通用，换任务时微调即可。

---

## 4. 目标检测的基本概念

### 4.1 任务与表示

分类：整张图是什么。检测：图中**有哪些物体、在哪、是什么**。输出一组 $(\text{类别},\text{置信度},\text{框})$。分割：每个像素属于什么。

框的两种常见表示：

- 角点式 $(x_1,y_1,x_2,y_2)$（左上、右下，像素坐标，原点在图像左上角，$x$ 向右、$y$ 向下）。
- 中心式 $(c_x,c_y,w,h)$。互转：$x_1=c_x-w/2,\ x_2=c_x+w/2$，$y$ 同理。

### 4.2 IoU（交并比）

**问题**：怎么衡量预测框与真值框"重合得好不好"？

$$\mathrm{IoU}=\frac{|A\cap B|}{|A\cup B|}=\frac{|A\cap B|}{|A|+|B|-|A\cap B|}\in[0,1]$$

**手算**（教学示例）：$A=[100,100,200,200]$，$B=[120,110,230,210]$。

- 交集宽 $=\min(200,230)-\max(100,120)=200-120=80$；高 $=\min(200,210)-\max(100,110)=200-110=90$；交集面积 $=7200$。
- $|A|=100\times100=10000$；$|B|=110\times100=11000$；并集 $=10000+11000-7200=13800$。
- $\mathrm{IoU}=7200/13800=0.5217$。

交集宽或高为负时取 0（不相交，IoU=0）。常用阈值 0.5：IoU≥0.5 算"检出"。

### 4.3 锚框（anchor）

**问题**：不同物体形状不同（行人高瘦，车辆扁宽）。让网络直接回归任意宽高很难收敛。
**做法**：预先设定几种典型形状（锚框），网络只预测相对锚框的**微调量**。锚框通常对训练集的真值框宽高做 K-means 聚类得到（YOLOv5 默认有一套 COCO 上聚类的锚框，并提供 AutoAnchor 对自己数据重新计算）。

YOLOv5 默认三组锚框（宽×高，像素，输入 640）：

| 特征层 | 步长 | 锚框 |
|---|---|---|
| P3 | 8 | (10,13) (16,30) (33,23) |
| P4 | 16 | (30,61) (62,45) (59,119) |
| P5 | 32 | (116,90) (156,198) (373,326) |

小步长（P3，特征图大）负责小物体，配小锚框；大步长（P5）负责大物体。

### 4.4 NMS（非极大值抑制）

**问题**：同一个物体周围会有许多重叠的高分框，需要只留一个。

**算法**（按类别分别做）：
1. 按分数降序排列。
2. 取分数最高的框，加入结果；
3. 删除与它 IoU 超过阈值的其余框；
4. 对剩下的重复 2–3，直到为空。

**手算**（教学示例）：5 个同类框，IoU 阈值 0.5。

| 框 | 坐标 | 分数 |
|---|---|---|
| A | [100,100,200,200] | 0.90 |
| B | [120,110,230,210] | 0.80 |
| C | [300,300,400,380] | 0.75 |
| D | [110,95,205,195] | 0.60 |
| E | [310,305,410,390] | 0.55 |

两两 IoU（按 4.2 的方法算）：AB=0.522，AD=0.781，BD=0.544，CE=0.692，其余为 0（不相交）。

- 取 A（0.90）：AB=0.522>0.5，删 B；AD=0.781>0.5，删 D；A 与 C、E 不相交，保留。
- 剩下 C、E：取 C（0.75）；CE=0.692>0.5，删 E。
- 结果：$\{A,C\}$。

若阈值改为 0.6：取 A 后，B（0.522<0.6）保留，D（0.781）删；再取 B，C 取后 E（0.692>0.6）删；结果 $\{A,B,C\}$。**阈值低：去重强但会误删相邻真实物体（拥挤场景漏检）；阈值高：保留更多重复框。**

变体：Soft-NMS（不直接删，而是降低重叠框的分数）、DIoU-NMS、Class-agnostic NMS（跨类别抑制）。YOLOv5 默认按类别做（实现中通过给不同类别的框加偏移使它们不会相交）。

### 4.5 AP 与 mAP

**问题**：检测要同时衡量"漏不漏"和"错不错"，并且不依赖某一个阈值。

**构造过程**（单个类别、IoU 阈值固定如 0.5）：

1. 把所有预测按置信度降序排列。
2. 逐个判断：与某个未被匹配的真值框 IoU≥阈值 → TP，否则 FP（重复检测同一真值也算 FP）。
3. 沿着排序累积 TP/FP，每个位置算 $P=\frac{TP}{TP+FP}$，$R=\frac{TP}{\text{真值总数}}$，得到 PR 曲线。
4. AP = PR 曲线下面积（先把曲线做"单调包络"：每个召回处的精确率取其右侧最大值）。
5. mAP = 各类别 AP 的平均。

**手算**（教学示例）：真值共 5 个，8 个预测按分数降序，TP 标志为 $[1,1,0,1,0,0,1,0]$。

| 排名 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| 累计 TP | 1 | 2 | 2 | 3 | 3 | 3 | 4 | 4 |
| Precision | 1.00 | 1.00 | 0.667 | 0.75 | 0.60 | 0.50 | 0.571 | 0.50 |
| Recall | 0.2 | 0.4 | 0.4 | 0.6 | 0.6 | 0.6 | 0.8 | 0.8 |

精确率包络：召回 0–0.2 取 1.0；0.2–0.4 取 1.0；0.4–0.6 取 0.75（右侧最大）；0.6–0.8 取 0.571；0.8–1.0 取 0（始终没达到 1.0 召回）。

$$AP=0.2\times1+0.2\times1+0.2\times0.75+0.2\times0.571+0.2\times0=0.2+0.2+0.15+0.1143+0=0.664$$

- **mAP@0.5**：IoU 阈值取 0.5。
- **mAP@0.5:0.95**（COCO 主指标）：IoU 阈值从 0.5 到 0.95、步长 0.05 共 10 档，各算 mAP 再平均，更严格地奖励框的定位精度。
- VOC 的 AP 用全点插值（如上），COCO 用 101 个召回点插值，算法细节略有差异，但思想相同。
- **mAP 是数据集级指标**；单个框的 confidence 只是一次检测的分数，不是准确率（与[机器学习基础](./01_机器学习基础.md)呼应）。

---

## 5. 两阶段与单阶段

| | 两阶段（R-CNN 系） | 单阶段（YOLO、SSD、RetinaNet） |
|---|---|---|
| 流程 | 先生成候选区域（RPN），再对每个候选分类+精修框 | 一次前向，直接在特征图每个位置预测类别和框 |
| 代表 | R-CNN → Fast R-CNN → Faster R-CNN → Mask R-CNN | YOLO 系列、SSD、RetinaNet |
| 精度 | 历史上更高，尤其小目标 | 早期略低，现已追平或超过 |
| 速度 | 较慢 | 快，适合实时与边缘 |
| 难点 | 流程复杂 | 正负样本极度不平衡（背景远多于物体） |

单阶段的正负样本不平衡问题：RetinaNet 用 **Focal Loss**（对容易分类的背景降权）；YOLO 用 objectness 分支与样本分配策略缓解。另有基于 Transformer 的 DETR 系，去掉锚框与 NMS，用集合预测，但训练更慢，边缘部署不如 YOLO 成熟。

---

## 6. YOLO 演进

> 以下是公开的、广为人知的演进脉络；具体数字（输出尺寸等）以各版论文/仓库为准，面试时讲"思想与差异"即可，**我在此未重新逐条核对原文**。

| 版本 | 年份 | 核心贡献 |
|---|---|---|
| YOLOv1 | 2016 | 把检测做成回归：图像分 $S\times S$ 网格，每格预测 $B$ 个框 + 类别概率；原论文 $S=7,B=2,C=20$，输出 $7\times7\times(2\times5+20)=7\times7\times30$。每格只预测一类，小目标和密集目标差 |
| YOLOv2 | 2017 | 加 BN；引入锚框（K-means 聚类得到）；多尺度训练；passthrough 融合细粒度特征 |
| YOLOv3 | 2018 | Darknet-53 骨干（带残差）；**三个尺度**预测（类 FPN），每尺度 3 个锚框；分类用独立 logistic（多标签） |
| YOLOv4 | 2020 | CSPDarknet53；SPP；PAN；Mosaic 增强；CIoU 损失；大量"bag of freebies / specials" |
| YOLOv5 | 2020 | Ultralytics 的 PyTorch 实现：工程化极好（训练、导出 ONNX、各规格 n/s/m/l/x）；自动锚框；坐标解码改为 $2\sigma-0.5$ |
| YOLOv8（后续） | 2023 | 无锚框（anchor-free）、解耦检测头、C2f 模块、去掉独立的 objectness，输出 $4+n_c$ 通道 |

**对比要点**：v5 输出每个预测含 $(x,y,w,h,\text{obj},\text{cls}_1..\text{cls}_{n_c})$ 即 $5+n_c$ 个数；v8 去掉 obj，只有 $4+n_c$。你的 CV 写的是 YOLOv5，所以以后者的格式为准，不要把 v8 的输出格式套到 v5 上。

---

## 7. YOLOv5 结构

### 7.1 总览

```text
输入 640x640x3 (letterbox 之后)
   |
[Backbone  CSPDarknet]    提特征，逐步下采样
   Focus/6x6 Conv(s=2) -> Conv(s=2) -> C3 -> Conv(s=2) -> C3 [P3, 80x80]
   -> Conv(s=2) -> C3 [P4, 40x40] -> Conv(s=2) -> C3 -> SPPF [P5, 20x20]
   |
[Neck  FPN + PAN]         多尺度特征融合
   P5 --上采样--> 与 P4 拼接 -> C3 --上采样--> 与 P3 拼接 -> C3 => 输出 N3 (80x80)
   N3 --Conv(s=2)--> 与上一级拼接 -> C3 => 输出 N4 (40x40)
   N4 --Conv(s=2)--> 与 P5 侧拼接 -> C3 => 输出 N5 (20x20)
   |
[Head  Detect]            三个尺度各一个 1x1 卷积
   N3 -> 3*(5+nc) 通道,  N4 -> 3*(5+nc),  N5 -> 3*(5+nc)
```

（以上是示意结构，各层通道数由配置文件的 `depth_multiple/width_multiple` 决定，n/s/m/l/x 不同规格用不同倍数缩放。）

### 7.2 各模块做什么

**Focus（v5.0 之前）**：把 $H\times W\times C$ 的图每隔一个像素切成四份（隔行隔列采样）再沿通道拼接，得到 $H/2\times W/2\times4C$，无信息损失地把分辨率换成通道，再接卷积。手算：$640\times640\times3\to320\times320\times12$。目的是**用更少的计算做第一次下采样**。v6.0 起 Ultralytics 用一个 $6\times6$、步长 2 的卷积替代 Focus（对 GPU 导出更友好、等价或更快）。你的模型是哪一版的结构，以你的权重/导出文件为准。

**CSP（C3 模块）**：输入特征沿通道分两路：一路经过若干个残差瓶颈块（Bottleneck），另一路直通；最后拼接并过 $1\times1$ 卷积。好处：梯度路径更丰富、参数和计算更少、特征复用。

**SPPF**：对同一特征连续做三次 $5\times5$ 最大池化（步长 1），把原始、一次、两次、三次池化结果拼接。等价于 SPP 的 $5/9/13$ 并行池化，但更快。作用：**融合多种感受野**，增强对不同大小目标的适应。

**FPN（Feature Pyramid Network，自顶向下）**：深层特征图小但语义强（知道"这是什么"），浅层特征图大但定位细（知道"在哪"）。FPN 把深层特征上采样后与浅层拼接，让浅层也获得语义信息，对小目标尤其重要。

**PAN（Path Aggregation Network，自底向上）**：FPN 之后再加一条自底向上的路径，把浅层的精确定位信息传回深层。FPN+PAN 使每个尺度的输出同时含有语义和定位信息。

**Head**：每个尺度一个 $1\times1$ 卷积，把通道数变为 $3\times(5+n_c)$（3 个锚框，每个预测 $5+n_c$ 个数）。

---

## 8. 输出张量与解码

### 8.1 形状

输入 $640\times640$ 时，三个尺度的网格为 $80\times80$（步长 8）、$40\times40$（16）、$20\times20$（32）。每个格点 3 个锚框：

$$3\times(80^2+40^2+20^2)=3\times(6400+1600+400)=25200$$

所以导出的 ONNX 若包含解码，常见输出形状为 $(1,25200,5+n_c)$。COCO 80 类：$5+80=85$，Head 的输出通道 $3\times85=255$。**你的类别数 $n_c$ 由 P4 的训练类别决定**。【待你补全：P4 模型的类别数、输入尺寸（640？320？）、是否导出了含解码的单输出，还是三路原始特征图】

很多面向 RKNN 的导出会**去掉解码部分**，输出三路原始特征图（形如 $(1,3\times(5+n_c),H,W)$ 或重排为 $(1,3,H,W,5+n_c)$），解码交给 CPU 上的后处理代码。这直接影响后面"在量化空间做后处理"（见下一章）。

### 8.2 解码公式（YOLOv5 v3.0 之后的形式）

网络对每个预测输出原始值 $t_x,t_y,t_w,t_h,t_o,t_{c_k}$，所在网格坐标 $(c_x,c_y)$（整数格索引），锚框 $(p_w,p_h)$（像素），步长 $\text{stride}$：

$$b_x=(2\sigma(t_x)-0.5+c_x)\cdot\text{stride},\qquad b_y=(2\sigma(t_y)-0.5+c_y)\cdot\text{stride}$$
$$b_w=p_w\cdot(2\sigma(t_w))^2,\qquad b_h=p_h\cdot(2\sigma(t_h))^2$$
$$\text{obj}=\sigma(t_o),\quad\text{cls}_k=\sigma(t_{c_k}),\quad\text{score}_k=\text{obj}\times\text{cls}_k$$

**为什么这样设计**：
- 中心偏移：$2\sigma-0.5\in(-0.5,1.5)$。允许中心落在本格之外半个格，便于相邻格都能负责边界上的物体，增加正样本、且梯度更好。（旧版 $\sigma(t_x)\in(0,1)$ 在边界处很难学。）
- 宽高：$(2\sigma)^2\in(0,4)$，宽高为锚框的 0 到 4 倍，有上界，避免旧版 $e^{t_w}$ 指数爆炸造成训练不稳。这也解释了锚框匹配阈值 4.0。
- 类别用独立 sigmoid（不是 softmax），支持多标签。

### 8.3 完整手算（教学示例数值）

在 $20\times20$ 层（P5，步长 32）的网格 $(c_x,c_y)=(9,5)$，使用锚框 $(116,90)$。设原始输出 $t_x=0.3,\ t_y=-0.2,\ t_w=0.5,\ t_h=0.1$，$t_o=1.5$，某类 $t_c=2.0$。

- $\sigma(0.3)=0.5744$，$\sigma(-0.2)=0.4502$，$\sigma(0.5)=0.6225$，$\sigma(0.1)=0.5250$
- $b_x=(2\times0.5744-0.5+9)\times32=(1.1489-0.5+9)\times32=9.6489\times32=308.76$
- $b_y=(2\times0.4502-0.5+5)\times32=(0.9003-0.5+5)\times32=5.4003\times32=172.81$
- $b_w=116\times(2\times0.6225)^2=116\times1.2449^2=116\times1.5498=179.78$
- $b_h=90\times(2\times0.5250)^2=90\times1.0499^2=90\times1.1022=99.22$
- 角点：$x_1=308.76-89.89=218.87$，$y_1=172.81-49.61=123.20$，$x_2=398.65$，$y_2=222.42$（相对 $640\times640$ 的 letterbox 图）
- 置信：$\sigma(1.5)=0.8176$，$\sigma(2.0)=0.8808$，$\text{score}=0.8176\times0.8808=0.720$

### 8.4 后处理流水线

```text
1. 把三路输出解码为 25200 个候选 (box, obj, cls...)
2. 过滤：obj < conf_thresh 的丢掉（先过滤，数量骤降）
3. 算 score = obj * cls；取最大类（或多标签）；score < 阈值丢弃
4. NMS（按类别），IoU 阈值常 0.45 左右（典型默认，以所用代码为准）
5. 坐标还原到原图：去掉 letterbox 补边、除以缩放比例
6. 输出 (class, score, box)
```

**letterbox 坐标还原**（手算）：原图 $1280\times720$ 缩放到 $640\times360$（比例 $0.5$），上下各补 $140$ 变成 $640\times640$。模型输出框的 $y=200$：先减去上补边 $200-140=60$，再除以比例 $60/0.5=120$。若漏做"去补边"，框整体向下偏 280 像素（$140/0.5$）。$x$ 方向没有补边，只需除以比例。

---

## 9. 损失函数

### 9.1 三项

$$L=\lambda_{box}L_{box}+\lambda_{obj}L_{obj}+\lambda_{cls}L_{cls}$$

典型默认权重约为 $\lambda_{box}=0.05,\lambda_{obj}=1.0,\lambda_{cls}=0.5$（会按类别数、输入尺寸缩放，且不同版本的超参文件略有不同；以你使用的仓库为准）。

### 9.2 框回归：从 IoU 到 CIoU

只用 L1/L2 回归 $(x,y,w,h)$ 的问题：数值误差相同，但 IoU 可能差很多，且框的大小不同尺度不一致。直接用 IoU 损失 $1-\mathrm{IoU}$，问题：不相交时 IoU=0，没有梯度。演进：

- **GIoU**：$\mathrm{GIoU}=\mathrm{IoU}-\dfrac{|C|-|A\cup B|}{|C|}$，$C$ 为包含两框的最小外接矩形。不相交时也有梯度。
- **DIoU**：加中心点距离惩罚 $\dfrac{\rho^2(b,b^{gt})}{c^2}$，$\rho$ 是两中心距离，$c$ 是外接矩形对角线长。收敛更快。
- **CIoU**：再加长宽比一致性惩罚：

$$L_{CIoU}=1-\mathrm{IoU}+\frac{\rho^2}{c^2}+\alpha v,\quad v=\frac4{\pi^2}\Big(\arctan\frac{w^{gt}}{h^{gt}}-\arctan\frac wh\Big)^2,\quad\alpha=\frac v{(1-\mathrm{IoU})+v}$$

YOLOv5 默认用 CIoU。

**手算**（用 4.2 的两个框，预测 $A$，真值 $B$）：
- GIoU：外接矩形 $C=[100,100,230,210]$，面积 $130\times110=14300$；$\mathrm{GIoU}=0.5217-\dfrac{14300-13800}{14300}=0.5217-0.0350=0.4868$。
- CIoU：中心 $A=(150,150)$，$B=(175,160)$，$\rho^2=25^2+10^2=725$；外接矩形对角线平方 $c^2=130^2+110^2=29000$；$\rho^2/c^2=0.025$。长宽比：$A$ 宽高 $100\times100$（比值 1），$B$ 宽高 $110\times100$（比值 1.1），$v=\frac4{\pi^2}(\arctan1-\arctan1.1)^2\approx0.00092$，$\alpha=v/((1-0.5217)+v)\approx0.0019$，所以 $\alpha v\approx1.8\times10^{-6}$，几乎可忽略。
 $L_{CIoU}=1-0.5217+0.025+0.0000018\approx0.5033$。（这里两框长宽比几乎一样，所以这一项很小；它在预测框"又细又长"而真值是方形时才起作用。）

### 9.3 objectness 与分类

- **objectness**：二元交叉熵。正样本的目标值不是 1，而是**预测框与真值框的 IoU**（detach，作为软标签），让 obj 分数反映定位质量；负样本目标为 0。不同尺度的 obj 损失加权不同（典型 $[4.0,1.0,0.4]$ 对应 P3/P4/P5），因为小尺度正样本多、需平衡。
- **分类**：独立 sigmoid 的二元交叉熵，只对正样本计算。
- 手算 obj 损失（一个正样本）：预测 $\sigma=0.8176$，目标 IoU$=0.52$：$-[0.52\ln0.8176+0.48\ln(1-0.8176)]=0.52\times0.2014+0.48\times1.7014=0.1047+0.8167=0.921$。

### 9.4 正样本分配（v5 的做法）

对每个真值框：
1. 计算它与每个锚框的宽高比 $\max(w/p_w,\ p_w/w,\ h/p_h,\ p_h/h)$，小于阈值 4.0 的锚框才算匹配。
2. 真值中心所在网格，加上最近的两个邻居格（根据中心落在格内的偏移 >0.5 还是 <0.5 选），一共 3 个格的对应锚框作为正样本。如中心在格内位置 $(0.7,0.3)$：右邻格和上邻格也成为正样本。

这一做法比 v3 的"一个真值只配一个最佳锚框"多出约 3 倍正样本，加快收敛，这与 $2\sigma-0.5$ 的偏移范围相呼应。

---

## 10. 数据增强

**为什么**：检测数据标注昂贵，增强是"免费"扩充；让模型对光照、尺度、位置变化不敏感。

| 方法 | 做法 | 作用/注意 |
|---|---|---|
| 色彩抖动（HSV） | 随机改色相/饱和/亮度 | 抗光照变化，对户外植物场景很重要 |
| 翻转 | 水平（有时垂直） | 注意是否破坏语义（带方向的物体） |
| 缩放/平移/旋转 | 随机仿射 | 尺度、位置不变性 |
| **Mosaic** | 把 4 张图拼成 1 张，框随之变换 | 一张图出现更多物体和上下文；等效更大 batch；对小目标友好 |
| **MixUp** | 两图按权重叠加 | 正则化，缓解过拟合 |
| Cutout/Copy-paste | 遮挡/粘贴物体 | 增加遮挡鲁棒性、稀有类样本 |
| Letterbox | 保比例缩放补边 | 训练和推理必须一致 |

**注意**：Mosaic 与真实分布有差异，所以训练末期常关闭 Mosaic 几个 epoch，让模型适应真实构图。增强只用在训练集，验证/测试/部署不加（letterbox 除外）。

---

## 11. 评估

1. **在带标注的验证/测试集上**算 precision、recall、mAP@0.5、mAP@0.5:0.95，按类别报告 AP，找出弱类。
2. **PR 曲线与 F1 曲线**：F1 随置信度阈值变化的曲线峰值给出推荐的置信度阈值，比拍脑袋的 0.25 或 0.5 靠谱。
3. 混淆矩阵：看类别间混淆与"背景被误检/物体被漏检"。
4. 不同尺寸（小/中/大）的 AP：小目标通常最差，提示是否需要更大输入或更细的特征层。
5. 速度：**FPS 与端到端延迟不同**。单看模型推理 20 ms 得到"50 FPS"，但整条链路（采集、预处理、推理、后处理、传输）加起来才是真实延迟（教学示例：10+5+20+5+10=50 ms，即 20 次/s）。
6. **域偏移**：用 COCO 验证集的 mAP 不能代表你温室/花园场景的表现；必须在真实场景的验证集上评估。
7. **定性检查**：空检测不能说明"没有病害"，可能是遮挡、失焦、光照差、漏检；决策逻辑要考虑这一点。

### 11.1 选置信度阈值的手算思路（教学示例）

在验证集上扫描置信度阈值，得到下表（数值为示例）：

| 阈值 | TP | FP | FN | Precision | Recall | F1 |
|---|---|---|---|---|---|---|
| 0.10 | 92 | 60 | 8 | 0.605 | 0.920 | 0.730 |
| 0.25 | 85 | 25 | 15 | 0.773 | 0.850 | 0.810 |
| 0.50 | 70 | 8 | 30 | 0.897 | 0.700 | 0.787 |
| 0.75 | 45 | 2 | 55 | 0.957 | 0.450 | 0.612 |

F1 在 0.25 附近最高。但**"F1 最高"不一定是你要的**：如果漏检一次病害的代价远大于误报一次，应偏向低阈值（高 recall）；如果每次误报都会触发一次浇水或打扰用户，则偏向高阈值。阈值是**业务决策**，用验证集数据支撑，而不是默认值。

### 11.2 小目标与输入分辨率

小目标（几十像素以内）在步长 32 的特征图上只占不到 1 个格子，信息在下采样中丢失。改善办法：增大输入分辨率（计算量约与分辨率平方成正比）、使用更细的特征层（P3 甚至 P2）、切片推理（SAHI 思路）、用更合适的锚框、增加小目标数据与 Mosaic 增强。部署在边缘设备时，分辨率受 NPU 算力和延迟约束，要在"能看见"和"跑得动"之间取舍。

### 11.3 类别不平衡与困难样本

背景远多于前景是检测的固有不平衡：25200 个预测里通常只有个位数是正样本。YOLOv5 用 objectness 分支并对正样本放大权重缓解；RetinaNet 用 Focal Loss $-\alpha(1-p_t)^\gamma\log p_t$ 压低容易样本的贡献（例如 $p_t=0.9,\gamma=2$ 时权重为 $0.01$，$p_t=0.1$ 时为 $0.81$，差 81 倍）。另一类方法是在线困难样本挖掘（OHEM）。数据层面：多采集"容易混淆"的背景（落叶、阴影、反光），并把误检样本加入训练集。

---

## 12. 边缘部署相关

衔接[下一章：量化与 RKNN 部署](./04_模型量化与边缘部署_RKNN.md)。本章为它铺垫的几点：

- **输入分辨率**是速度/精度的主要旋钮。分辨率减半，计算量约降到 1/4；小目标会受影响更大。
- **激活函数与量化**：SiLU 含 sigmoid，INT8 量化下比 ReLU/LeakyReLU 更难处理，边缘部署中常有人把激活换成 ReLU 系重新训练，或依赖工具链的算子支持。
- **导出时去掉解码**：把后处理中的 sigmoid、乘锚框、网格偏移放到 CPU 做，NPU 只算卷积部分，减少 NPU 不支持的算子，也便于在量化空间做优化。
- **NMS 通常在 CPU**：候选数（25200）经阈值过滤后才进入 NMS，复杂度大致 $O(n^2)$，所以**先过滤再 NMS**。
- **预处理必须与训练一致**：通道顺序（RGB/BGR）、归一化（除以 255 还是减均值除方差）、letterbox 补边的颜色（114 灰）。
- **模型规格**：n/s 规格适合 NPU 边缘端，参数量大致在百万到千万量级（具体数值以官方仓库表格为准，我未核对当前数值）。

---

## 13. 在你的项目里怎么用

**P4**：你的 CV 写的是"在 RK3588 上搭建 YOLOv5/RKNN INT8 推理链，把相机帧转成带置信度与时间戳的结构化观测"。对应关系：

| 本章知识 | P4 中的位置 |
|---|---|
| letterbox、坐标还原 | 预处理与后处理的两端；坐标不还原 → 框整体偏移 |
| 输出张量解码、阈值过滤、NMS | 后处理代码 |
| confidence 不是准确率 | 结构化观测里的 confidence 字段只是模型分数，决策层要谨慎使用 |
| mAP/PR/F1 | 选阈值、评估模型质量 |
| 空检测 ≠ 健康 | 决策链里不能把"没检测到"当"没问题" |

项目入口：[P4 OpenHarmony 智能园艺系统](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)，量化与板端细节见[下一章](./04_模型量化与边缘部署_RKNN.md)。

诚实边界：训练数据集、类别列表、训练细节、验证指标，你如果没有亲手做，就说"I used a trained model and built the deployment and post-processing chain"。
【待你补全：P4 检测的是哪些类别（病斑？植物？）；模型权重来源；是否在真实场景上评估过 precision/recall；NMS 与置信度阈值的取值及选取依据】

---

## 14. 常见错误与故障现象

| 现象 | 原因 | 排查 |
|---|---|---|
| 框整体偏移/缩放不对 | 没还原 letterbox 补边或比例 | 手算一个框验证 |
| 红蓝颠倒，检测变差 | RGB/BGR 与训练不一致 | 对齐通道顺序 |
| 大量重复框 | NMS 阈值太高或没做 | 调 IoU 阈值、确认按类做 |
| 拥挤目标被漏 | NMS 阈值太低 | 提高阈值、用 Soft-NMS |
| 输出全为很小分数 | 归一化错（0–255 vs 0–1）、NCHW/NHWC 错 | 对照转换脚本的 mean/std 与布局 |
| 量化后精度掉很多 | 校准集不代表现场；激活范围被异常值撑大 | 见下一章 |
| 把 v8 的输出格式套给 v5 | 通道含义不同（有无 obj） | 先打印输出形状确认 |
| 用 confidence 当准确率 | 概念错误 | 在标注集上统计 |

---

## 15. 面试问答（英文）

**Q1. Explain how YOLO works.**

> YOLO is a one-stage detector. A single forward pass over the image produces predictions at every cell of a few feature maps. For YOLOv5, each cell on three scales predicts three anchor-based boxes, and each prediction contains box offsets, an objectness score and class scores. I then filter by confidence, decode the boxes, and apply non-maximum suppression to remove duplicates. It is fast because there is no separate region-proposal stage.

中文要点：单阶段；三尺度每格 3 个锚框；每个预测含框、obj、类别；过滤→解码→NMS。
追问：*How many predictions at 640×640?* — $3\times(80^2+40^2+20^2)=25200$。

**Q2. What is IoU and NMS? Walk me through an example.**

> IoU is intersection over union of two boxes. NMS sorts the boxes by score, keeps the best one, and removes any other box whose IoU with it exceeds a threshold, then repeats. For example, two boxes with IoU 0.52 at a 0.5 threshold: the lower-scoring one is removed. A threshold that is too low can delete real neighbouring objects in crowded scenes.

**Q3. What is the difference between mAP@0.5 and mAP@0.5:0.95?**

> mAP@0.5 counts a detection as correct when IoU is at least 0.5. mAP@0.5:0.95 averages over IoU thresholds from 0.5 to 0.95, so it also rewards accurate localization. A model can look good at 0.5 but drop a lot on the stricter metric.

**Q4. Is the confidence score the accuracy of the detection?**

> No. Confidence is a score from the model for one detection, and it is often not calibrated. Accuracy-type quantities like precision, recall and mAP are computed over a labelled dataset. In my project I treat confidence as evidence, not as ground truth, and I record it with a timestamp so later decision logic can check freshness.

**Q5. Why the FPN and PAN structure in the neck?**

> Deep features carry strong semantics but low resolution, shallow features carry precise location but weak semantics. FPN passes semantics down to shallow layers, PAN passes localization back up, so every output scale has both.

**Q6. Anchor-based versus anchor-free detection?**

> Anchor-based detectors predict offsets relative to preset box shapes, which makes regression easier and gives prior knowledge about object sizes, but anchors are hyperparameters that must match the dataset and they add many hand-designed choices. Anchor-free detectors, like YOLOv8, predict box geometry directly from locations, which simplifies the pipeline and removes anchor tuning, at the cost of needing a good label-assignment strategy. My project uses YOLOv5, which is anchor-based.

**Q7. How do you choose the confidence threshold for deployment?**

> I sweep the threshold on a labelled validation set, plot precision, recall and F1, and choose according to the cost of errors. In an irrigation or monitoring context a missed detection and a false alarm have different costs, so I would not simply take the default value. I would also treat the detection as evidence together with other signals rather than as a ground truth.

**Q8. What did you personally do in the vision part of your project?**（要如实）

> I built the edge inference chain on the RK3588: preprocessing, running the quantized YOLOv5 model through RKNN, post-processing with thresholding and NMS, and turning the result into a structured observation with confidence and a timestamp. 【请按你的实际训练参与程度补充一句】

---

## 16. 自测题

1. 输入 $224\times224$，核 $7\times7$，$s=2$，$p=3$，输出尺寸？
   **答案**：$\lfloor(224+6-7)/2\rfloor+1=\lfloor111.5\rfloor+1=112$。
2. 卷积层 $64\to128$ 通道，$3\times3$ 核，带偏置，参数量？
   **答案**：$128\times(64\times9+1)=73856$。
3. 计算框 $[0,0,4,4]$ 与 $[2,2,6,6]$ 的 IoU。
   **答案**：交集 $2\times2=4$；并集 $16+16-4=28$；IoU $=1/7=0.143$。
4. 输入 $416\times416$，三个尺度（步长 8/16/32），每格 3 锚框，预测总数？
   **答案**：网格 $52,26,13$；$3\times(2704+676+169)=3\times3549=10647$。
5. YOLOv5 解码中为什么宽高用 $(2\sigma)^2$ 而不是 $e^t$？
   **答案**：有上界（0–4 倍锚框），训练稳定；与锚框匹配阈值 4 对应。
6. NMS 之前为什么先按 conf 过滤？
   **答案**：25200 个候选绝大多数是背景，先过滤使 NMS 的 $O(n^2)$ 开销很小。
7. 为什么 mAP 不能用来解释单个框的 confidence？
   **答案**：mAP 是整个数据集、按排序统计的指标；confidence 是单次输出的分数，且未必校准。
8. 阈值 0.5 的混淆数据 TP=70、FP=8、FN=30，求 precision、recall、F1。
   **答案**：$P=70/78=0.897$；$R=70/100=0.70$；$F1=2\times0.897\times0.7/1.597=0.787$。
9. Focal Loss 中 $\gamma=2$，$p_t=0.95$ 与 $p_t=0.2$ 的调制因子 $(1-p_t)^\gamma$？
   **答案**：$0.05^2=0.0025$；$0.8^2=0.64$，相差约 256 倍，容易样本几乎被忽略。
10. 瓶颈块 $1\times1$（128→32）+ $3\times3$（32→32）+ $1\times1$（32→128）参数量？
   **答案**：$4096+9216+4096=17408$（忽略偏置）；直接 $3\times3$ 128→128 为 $147456$，约 $1/8.5$。

---

## 本章要点速记

1. 卷积 = 局部连接 + 权重共享；输出尺寸 $\lfloor(H+2p-k)/s\rfloor+1$，参数 $C_{out}(C_{in}k^2+1)$。
2. 感受野递推 $r_l=r_{l-1}+(k_l-1)j_{l-1}$；两个 $3\times3$ 等效 $5\times5$ 且更省。
3. ResNet 的恒等通路解决深层退化；MobileNet 深度可分离卷积把计算降到约 1/8；CSP 减少重复梯度。
4. IoU=交/并；NMS：降序、取最高、删高重叠、循环；阈值是漏检与重复之间的权衡。
5. AP = PR 曲线下面积（精确率包络），mAP 对类别平均；mAP@0.5:0.95 更看重定位。
6. 单阶段一次前向直出结果，快；两阶段先候选后精修，历史上更准。
7. YOLO 演进：v1 网格回归 → v2 锚框 → v3 三尺度 → v4 CSP/PAN/Mosaic/CIoU → v5 工程化 → v8 无锚框、无 obj。
8. YOLOv5：Backbone(CSP, SPPF) + Neck(FPN+PAN) + Head(3 尺度)；640 输入共 25200 个预测，每个 $5+n_c$ 个数。
9. 解码：中心 $(2\sigma-0.5+c)\cdot s$，宽高 $p\cdot(2\sigma)^2$，score=obj×cls；先过滤、再 NMS、最后还原 letterbox。
10. 损失 = CIoU 框 + obj(BCE，目标为 IoU) + cls(BCE)；confidence 不是准确率，评估看 mAP/PR/F1。

## 相关链接

- 上一章：[02 神经网络与反向传播](./02_神经网络与反向传播.md)
- 下一章：[04 模型量化与边缘部署 RKNN](./04_模型量化与边缘部署_RKNN.md)
- 相关项目：[P4 OpenHarmony 智能园艺系统](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)
- 八股速查：[计算机视觉与目标检测八股](../../03_八股知识/02_人工智能/03_计算机视觉与目标检测八股.md)
