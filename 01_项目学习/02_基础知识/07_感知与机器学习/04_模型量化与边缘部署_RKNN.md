# 模型量化与边缘部署 RKNN

> **一句话定位**：把在 GPU 上用 FP32 训练好的 YOLO 模型，压缩成 INT8、放到 RK3588 的 NPU 上实时跑，并让精度损失可控、可解释、可度量。这是你 P4 视觉链最"工程"的一环，也是面试官最容易追问细节的一环。

**本章学完你能……**

- 写出 FP32→INT8 的量化/反量化公式，解释 scale、zero point 的含义，手算对称与非对称量化、舍入误差与饱和误差。
- 说清逐层（per-tensor）与逐通道（per-channel）量化的区别，并手算一个让逐通道更准的例子。
- 解释 PTQ 与 QAT 的原理与取舍，校准集为什么重要，min-max、百分位、KL 校准的区别。
- 手算一次 INT8 矩阵乘加的完整过程（INT32 累加、重量化）。
- 复述 RK3588 NPU 与 RKNN-Toolkit 的转换流程：ONNX→RKNN→板端推理，配置 mean/std、校准数据集、letterbox 的要点。
- 解释"在量化空间做后处理阈值过滤"的优化是什么、为什么成立。
- 设计性能与精度评估：FP32 对比 INT8、逐层余弦相似度、单帧延迟与端到端延迟的区别。
- 用英文回答"Why quantize? What are the trade-offs? How did you deploy YOLO on RK3588?"

**前置知识**

- [03 卷积网络与目标检测 YOLO](./03_卷积网络与目标检测_YOLO.md)（YOLOv5 输出与解码、letterbox）
- [02 神经网络与反向传播](./02_神经网络与反向传播.md)（BN、激活、梯度）
- 平台背景：[OpenHarmony / Hi3861 / RK3588 平台基础](../08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)

**目录**

1. 为什么要量化
2. 量化公式与符号
3. 对称与非对称
4. 逐层与逐通道
5. 校准与饱和误差
6. INT8 运算怎么算
7. PTQ 与 QAT
8. RK3588 NPU 与 RKNN 工具链
9. ONNX→RKNN 转换流程与配置
10. 预处理：mean/std 与 letterbox
11. 后处理在量化空间的优化
12. 性能与精度评估
13. 在你的项目里怎么用
14. 常见错误与故障现象
15. 面试问答
16. 自测题

---

## 1. 为什么要量化

### 1.1 它要解决什么问题

边缘设备（RK3588 这类板子）算力、内存带宽、功耗都有限。FP32 模型每个权重 4 字节，计算单元也大且耗电。**量化**：用低位宽整数（常用 INT8）近似浮点数，换取：

| 收益 | 解释（教学示例数值） |
|---|---|
| 模型变小 | 1000 万参数：FP32 约 40 MB，INT8 约 10 MB（4 倍） |
| 内存带宽下降 | 同样的数据搬运量变成 1/4；推理很多时候卡在带宽而不是算力 |
| 计算更快、更省电 | 整数乘加单元面积小、能耗低，NPU 并行度更高 |
| 硬件支持 | 很多 NPU 的峰值算力指标就是按 INT8 给出的 |

代价：精度可能下降（舍入误差、饱和误差），需要校准、评估，必要时对敏感层保留更高精度。

### 1.2 量化 ≠ 训练

> **面试要点**：量化/转换是**部署阶段**的格式与数值精度变换，输入是已有权重，需要的是校准数据，不需要标签和损失（PTQ）。它不等于重新训练模型。有一个转换脚本，**不能**单独证明你训练过模型。

---

## 2. 量化公式与符号

### 2.1 仿射量化

浮点值 $x$ 映射到整数 $q$：

$$q=\mathrm{clip}\Big(\mathrm{round}\Big(\frac xs\Big)+z,\ q_{\min},\ q_{\max}\Big)$$

反量化（近似还原）：

$$\hat x=s\,(q-z)$$

符号：

- $s>0$：scale（缩放因子，一个整数步长代表多大的浮点量）。**单位同 $x$**。
- $z$：zero point（零点，浮点 0 对应的整数）。
- $[q_{\min},q_{\max}]$：整数范围。INT8 为 $[-128,127]$，UINT8 为 $[0,255]$。
- $\mathrm{round}$ 四舍五入（硬件实现可能是"偶数舍入"等细节，边界值如 $x/s=2.5$ 要小心）。

### 2.2 两种误差

1. **舍入误差**：只要不饱和，$|\hat x-x|\le s/2$。$s$ 越小越准。
2. **饱和（截断）误差**：$x$ 超出可表示范围 $[s(q_{\min}-z),\,s(q_{\max}-z)]$ 被 clip，误差可以任意大。

**权衡**：范围覆盖宽（$s$ 大）→ 不饱和但分辨率差；范围窄（$s$ 小）→ 分辨率好但大值被截断。校准的任务就是选一个好的范围。

### 2.3 手算（教学示例数值）

$s=0.1,\ z=0$：

- $x=0.26$：$x/s=2.6\to q=3\to\hat x=0.3$，误差 $+0.04\le s/2=0.05$ ✓。
- $x=20$：$x/s=200\to$ clip 到 $127\to\hat x=12.7$，误差 $-7.3\gg s/2$ ——**饱和误差**。

---

## 3. 对称与非对称量化

### 3.1 定义

- **对称**：$z=0$，范围关于 0 对称 $[-\alpha,\alpha]$，$s=\alpha/127$（INT8 常取 $[-127,127]$，保持对称）。优点：硬件实现简单（乘加里不用处理零点项）。
- **非对称**：$z\ne0$，范围 $[x_{\min},x_{\max}]$ 任意，$s=(x_{\max}-x_{\min})/(q_{\max}-q_{\min})$，$z=\mathrm{round}(q_{\min}-x_{\min}/s)$。能充分利用全部 256 个整数级别。

### 3.2 非对称手算（UINT8，教学示例）

数据范围 $[-1.0,\ 3.0]$：

- $s=4.0/255=0.015686$
- $z=\mathrm{round}(0-(-1.0)/0.015686)=\mathrm{round}(63.75)=64$

| $x$ | $x/s$ | $q=\mathrm{round}(x/s)+64$ | $\hat x=s(q-64)$ | 误差 |
|---|---|---|---|---|
| $-1.0$ | $-63.75$ | 0 | $-1.0039$ | $-0.0039$ |
| 0.0 | 0 | 64 | 0 | 0 |
| 0.26 | 16.575 | 81 | 0.2667 | $+0.0067$ |
| 1.5 | 95.625 | 160 | 1.5059 | $+0.0059$ |
| 3.0 | 191.25 | 255 | 2.9961 | $-0.0039$ |
| 3.5 | 223.1 | 255（clip） | 2.9961 | $-0.504$ |

注意两点：(1) **浮点 0 被精确表示**为 $q=z=64$（这很重要，因为 0 填充、ReLU 输出的 0 要无误差）；(2) 3.5 超出范围被饱和。

### 3.3 对称手算（INT8，教学示例）

对称范围 $\alpha=3$：$s=3/127=0.023622$。

- $x=0.26$：$x/s=11.007\to q=11\to\hat x=0.2598$，误差 $-0.0002$。
- $x=-1.0$：$-42.33\to-42\to-0.9921$。
- $x=5.0$：$211.7\to$ clip $127\to3.0$，误差 $-2.0$（饱和）。

### 3.4 什么时候用哪种

- **权重**：近似零均值对称，常用对称（可逐通道）。
- **激活**：ReLU/SiLU 之后几乎全是非负，范围 $[0,6]$。对称 INT8 只用到正半边 $0..127$，步长 $6/127=0.0472$；非对称 UINT8 用满 $0..255$，步长 $6/255=0.0235$，**分辨率高一倍**。所以激活更适合非对称。
- RKNN 一类工具链的默认方案通常是非对称 8 位量化，也支持其他类型，**具体的 dtype 选项名称随 Toolkit 版本变化**，以你所用版本的文档为准。

---

## 4. 逐层与逐通道量化

### 4.1 问题

卷积权重形状 $[C_{out},C_{in},k_h,k_w]$。不同输出通道的权重幅度可能差几倍到几十倍。逐层（per-tensor）：整个张量共用一个 $s$，由最大幅度通道决定，小幅度通道只能用到很少的几个整数级别，被"稀释"。逐通道（per-channel）：每个输出通道有自己的 $s_c$（和 $z_c$）。

### 4.2 手算（教学示例）

两个输出通道：通道 1 权重范围 $[-0.5,0.5]$，通道 2 范围 $[-2.0,2.0]$；考察通道 1 中的权重 $w=0.1$（对称 INT8）。

- 逐层：$s=2.0/127=0.015748$，$q=\mathrm{round}(0.1/0.015748)=\mathrm{round}(6.35)=6$，$\hat w=0.0945$，误差 $0.0055$（相对 5.5%）。
- 逐通道：$s_1=0.5/127=0.003937$，$q=\mathrm{round}(25.4)=25$，$\hat w=0.0984$，误差 $0.0016$（相对 1.6%）。

逐通道误差约为逐层的 $1/3$，因为通道 1 用上了约 4 倍的整数级别。

### 4.3 为什么激活不逐通道

逐通道量化权重很容易：不同输出通道的结果最后是各自累加，换一个 $s_c$ 只影响该通道的重量化系数。但**激活逐通道**会让同一次卷积里相加的不同输入通道有不同的 scale，累加之前就要先统一，破坏整数矩阵乘的结构。因此常见做法是：**权重逐通道，激活逐层（per-tensor）**。RKNN 官方文档也有类似说明（你的学习笔记里引用过，具体以所用 Toolkit 版本文档为准）。

---

## 5. 校准与饱和误差

### 5.1 为什么需要校准

权重是固定的，范围直接可知。**激活的范围取决于输入数据**，所以量化前要用一批有代表性的样本（校准集）跑一遍浮点模型，统计各层激活的分布，据此决定每层的 $s,z$。

### 5.2 范围选择方法

| 方法 | 做法 | 特点 |
|---|---|---|
| min-max | 取观测到的最小/最大值 | 简单，但被离群值撑大范围，分辨率变差 |
| 百分位 | 取 99.9% 分位等 | 截掉极端值，主体更精细 |
| KL 散度/熵校准 | 选阈值使量化前后分布的 KL 散度最小 | 自动在舍入与饱和之间折中 |
| MSE/MMSE | 选阈值使量化误差均方最小 | 类似 |

### 5.3 手算：离群值如何破坏精度（教学示例）

一层激活的 10 个值：$[0.1,0.2,0.3,0.25,0.15,0.4,0.35,0.2,0.3,12.0]$，其中 12.0 是个离群值。

**方案 A：min-max 对称**，$\alpha=12$，$s=12/127=0.0945$。主体值量化后：

| 原值 | 0.1 | 0.2 | 0.3 | 0.25 | 0.15 | 0.4 | 0.35 | 0.2 | 0.3 |
|---|---|---|---|---|---|---|---|---|---|
| $q$ | 1 | 2 | 3 | 3 | 2 | 4 | 4 | 2 | 3 |
| $\hat x$ | 0.094 | 0.189 | 0.283 | 0.283 | 0.189 | 0.378 | 0.378 | 0.189 | 0.283 |

主体值只剩 1–4 共 4 个整数级别，相对误差高达 10–20%（如 0.15→0.189，误差 +0.039）。

**方案 B：截断在 $\alpha=0.4$**，$s=0.4/127=0.00315$：主体值误差都在 0.002 以内，但 12.0 被饱和成 0.4，误差 $-11.6$。

哪个更好取决于离群值是否重要。如果那个 12.0 是少数异常像素，B 更好；如果它是关键的强响应（比如目标检测里最高分的位置），截断会损伤结果。这就是为什么**需要在代表性数据上评估最终指标**，而不是盲选。

### 5.3b 为什么有些激活特别难量化

- **激活范围分布重尾**：检测头附近的输出，大多数位置是"背景"（很小的值），少数目标位置有很大响应。min-max 会被少数大响应撑大范围，背景值被粗糙量化。对 sigmoid 之前的 logit，大负数区间（背景）精度不重要，所以影响通常较小，但对 box 回归值（需要精细）则要谨慎。
- **SiLU/Swish**：$x\sigma(x)$ 在负半轴有一个小凹陷，最小值约 $-0.278$，正半轴近似线性。对称量化会浪费范围，非对称更合适；NPU 上 SiLU 还需要用查表近似，带来额外误差。这就是边缘部署中常见"把激活换成 ReLU/LeakyReLU 再训练"的原因之一。
- **残差相加**：两路 scale 不同，需要重新对齐，会引入额外舍入。
- **输出层与第一层**：输入层面对的是原始像素分布，输出层直接决定检测框精度，这两处常是掉点的"重灾区"，混合量化优先考虑它们。

### 5.3c 误差如何逐层累积（定性）

每一层都会引入量化噪声，后面的层把前面的噪声当作输入的一部分继续传递，并可能被放大或衰减。网络越深、越窄、权重分布越极端，累积越严重。轻量网络（参数少、通道少、每个通道承载的信息多）比大网络对量化更敏感，这也是 MobileNet 类网络 PTQ 掉点常比 ResNet 大的原因。因此**不要用某个大模型的量化经验推断自己的小模型**，要用自己的模型和数据评估。

### 5.4 校准集要求

- **代表真实部署数据**：同样的相机、光照、背景、植物种类、距离。只用不相关图片，统计的范围与现场不符。
- **数量**：不用多，通常几十到几百张足够估计分布（RKNN 文档给出的量级是 20–200 张，具体以文档为准）。
- **预处理必须和部署完全一致**：同样的 letterbox、通道顺序、归一化。不一致 = 统计了另一个分布。
- 涵盖正常与困难样本（逆光、遮挡、无目标的背景图）。

---

## 6. INT8 运算怎么算

### 6.1 原理

量化后线性层 $y=\sum_iw_ix_i$ 变为（对称、无偏置时）：

$$y\approx\sum_i(s_wq_{w,i})(s_xq_{x,i})=s_ws_x\sum_iq_{w,i}q_{x,i}$$

核心整数运算是 $\sum q_wq_x$：INT8×INT8 的乘积最大 $127\times127=16129$，要求累加用 **INT32** 累加器，否则溢出。然后乘以 $s_ws_x$ 还原；若下一层也需要 INT8 输入，则再除以输出 scale 做**重量化（requantization）**。硬件上这个比例 $M=s_ws_x/s_o$ 被近似为"定点乘法 + 移位"，避免浮点运算。

### 6.2 完整手算（教学示例数值）

输入 $x=[0.5,-0.9,2.0]$，权重 $w=[0.2,0.4,-0.1]$。真实结果 $0.1-0.36-0.2=-0.46$。

量化参数（对称）：$s_x=2.0/127=0.015748$，$s_w=0.5/127=0.003937$。

- $q_x$：$0.5/0.015748=31.75\to32$；$-0.9/0.015748=-57.15\to-57$；$2.0\to127$ → $[32,-57,127]$
- $q_w$：$0.2/0.003937=50.8\to51$；$0.4\to101.6\to102$；$-0.1\to-25.4\to-25$ → $[51,102,-25]$
- INT32 累加：$32\times51+(-57)\times102+127\times(-25)=1632-5814-3175=-7357$
- 还原：$-7357\times s_xs_w=-7357\times6.2\times10^{-5}\approx-0.4561$（精确：$-0.4561$），与真值 $-0.46$ 的误差 $0.004$。
- 若输出 scale $s_o=0.5/127=0.003937$：$M=s_xs_w/s_o=0.015748$，$-7357\times M=-115.86\to q_o=-116$，反量化 $-116\times0.003937=-0.4567$。

要点：**大部分计算在整数域完成，误差来自输入/权重的量化，累加本身是精确的**。偏置通常以 INT32 保存，其 scale 为 $s_ws_x$。

### 6.3 其他算子

- 逐元素加（残差连接）：两个输入 scale 不同要先对齐。
- sigmoid/SiLU 等非线性：常用查表（LUT）或分段近似，是量化误差的一个来源。
- Conv+BN+激活：先把 BN 融合进卷积（见上一章 8.4），再量化。融合后的权重分布可能变化，更需要逐通道。

### 6.4 量化噪声的粗略量级

均匀量化的误差近似均匀分布，方差 $s^2/12$。位宽每增加 1 位，信噪比大约提高 6 dB（8 位约 50 dB 量级）。这说明为什么 8 位对多数卷积网络够用，而 4 位以下需要 QAT 等更精细手段。

---

## 7. PTQ 与 QAT

| | PTQ（训练后量化） | QAT（量化感知训练） |
|---|---|---|
| 做法 | 拿训练好的 FP32 模型 + 校准集，直接确定 $s,z$ | 训练/微调时在前向插入"伪量化"节点，让网络适应量化误差 |
| 需要 | 少量无标签数据 | 完整训练流程、标签、算力 |
| 成本 | 低，分钟级 | 高 |
| 精度 | 8 位通常够；低位宽、轻量模型掉点明显 | 更高，接近 FP32 |
| 典型用途 | 快速部署（RKNN 的主要方式） | PTQ 掉点不可接受时 |

### 7.1 QAT 的伪量化与 STE

前向：$x_{fq}=s\cdot\mathrm{clip}(\mathrm{round}(x/s),q_{\min},q_{\max})$（输出仍是浮点，但取值被限制在量化格点上）。问题：$\mathrm{round}$ 的导数几乎处处为 0，梯度传不回去。解决：**直通估计器（STE）**——反向时假装 round 不存在：范围内梯度原样通过（$\partial x_{fq}/\partial x=1$），被 clip 的部分梯度为 0。手算：$s=0.1$，$x=0.26$：前向 $x_{fq}=0.3$，反向系数 1；$x=13$（被 clip 到 12.7）：反向系数 0。

### 7.2 混合精度

检测头、第一层、最后一层往往对量化最敏感。常见策略：这些层保留 FP16，其余层 INT8。RKNN-Toolkit2 提供混合量化的流程（具体接口以版本文档为准）。排查思路：**逐层对比 FP32 与量化后的输出（余弦相似度），找出误差大的层**。

---

## 8. RK3588 NPU 与 RKNN 工具链

> 平台规格数据来自公开资料的常见说法；**我没有在本次编写中重新打开官方文档核对**，面试前请以 Rockchip 官方资料为准。

### 8.1 硬件

RK3588 是 8 核 ARM（4×Cortex-A76 + 4×Cortex-A55）SoC，内置 NPU，官方宣称的 INT8 峰值约 6 TOPS（常描述为三个核心）。NPU 支持 INT8/INT16/FP16 等类型。**TOPS 是峰值**：1 次乘加（MAC）= 2 次运算。

教学示例：若某模型是 10 GFLOPs（5 G 次乘加），6 TOPS 对应的理论上限约 $6\times10^{12}/10^{10}=600$ 次/s。实际远低于此：受内存带宽、算子利用率、NPU 不支持的算子回落到 CPU、前后处理等限制。**不要用峰值算力推算实测帧率**，应实测。

### 8.2 工具链

| 组件 | 运行位置 | 作用 |
|---|---|---|
| RKNN-Toolkit2 | PC（Linux x86） | 模型转换（ONNX 等→RKNN）、量化、模拟器推理、精度分析、性能评估 |
| RKNPU 驱动 + 运行时 librknnrt | 板端 | 加载 `.rknn` 模型并驱动 NPU |
| RKNN-Toolkit-Lite2 / C API (`rknn_api`) | 板端 | Python 或 C/C++ 推理接口 |
| rknn_model_zoo | 参考 | 官方示例（含 YOLOv5 的转换与后处理代码），可作为接口与后处理参考 |

关键约束：**转换工具版本、板端运行时版本、驱动版本需要匹配**。版本不匹配是"模型加载失败/结果乱"的常见根因。

### 8.3 整体流程

```text
 (训练环境, GPU)                   (PC, RKNN-Toolkit2)                      (板端 RK3588)
 PyTorch YOLOv5 权重 .pt
        |  export
        v
     ONNX 模型 -------> config(mean,std,target) --> load_onnx --> build(INT8, 校准集)
                                                                    |
                                                                    v
                                                             export_rknn -> model.rknn
                                                                    |  拷贝
                                                                    v
                                      camera -> 预处理(letterbox,RGB,uint8) -> NPU推理 -> 三路INT8输出
                                                -> 后处理(阈值,解码,NMS,还原坐标) -> 结构化观测
```

---

## 9. ONNX→RKNN 转换流程与配置

### 9.1 导出 ONNX 的注意点

- 固定输入尺寸（如 $1\times3\times640\times640$），NPU 偏好静态形状。
- 选定 opset，保证目标算子被 Toolkit 支持。
- 是否导出解码部分：见 [YOLO 章 8.1](./03_卷积网络与目标检测_YOLO.md)，面向 NPU 常去掉解码，仅输出三路原始特征，后处理交给 CPU。

### 9.2 转换脚本骨架（教学示例，非项目源码；API 名称以所用版本文档为准）

```python
from rknn.api import RKNN

rknn = RKNN(verbose=True)

# 1) 配置：预处理的 mean/std 与目标平台
rknn.config(mean_values=[[0, 0, 0]],
            std_values=[[255, 255, 255]],
            target_platform='rk3588')

# 2) 载入 ONNX
rknn.load_onnx(model='best.onnx')

# 3) 构建：开启量化，给校准集清单（每行一张图片路径）
rknn.build(do_quantization=True, dataset='dataset.txt')

# 4) 导出
rknn.export_rknn('model.rknn')

# 5)（可选）在 PC 上用模拟器或连板推理、做精度分析与性能评估
# rknn.init_runtime(...); outputs = rknn.inference(inputs=[img])
# rknn.accuracy_analysis(inputs=['sample.jpg'], output_dir='./snapshot')
# rknn.eval_perf()
rknn.release()
```

### 9.2b 板端推理骨架（教学示例，非项目源码；API 以版本文档为准）

```python
import cv2
import numpy as np
from rknnlite.api import RKNNLite            # 板端轻量运行时

rknn = RKNNLite()
rknn.load_rknn('model.rknn')
rknn.init_runtime()                           # 可选择使用哪个 NPU 核心，见版本文档

img = cv2.imread('frame.jpg')                 # BGR
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)    # 与训练/转换时一致
img, ratio, (pad_w, pad_h) = letterbox(img, (640, 640))   # 保比例缩放+补边（自行实现）
inputs = [np.expand_dims(img, 0)]             # (1, 640, 640, 3), uint8, 不要再除以 255
outputs = rknn.inference(inputs=inputs)       # 三路原始特征图（若导出时去掉了解码）
# ... 后处理：阈值过滤 -> 解码 -> NMS -> 用 ratio/pad 还原到原图坐标 ...
rknn.release()
```

三点：(1) 输入是 **uint8 的原始像素**，布局以模型要求为准（常见 NHWC）；(2) 反量化、解码是你自己的后处理代码；(3) 一帧的耗时要分成"采集、预处理、NPU、后处理"分段计时。RK3588 有多个 NPU 核心，工程上可以用多线程/多进程把不同帧分给不同核心提高吞吐，但这是**吞吐优化，不降低单帧延迟**。

### 9.3 `mean_values` / `std_values` 到底是什么

工具链把"归一化"**写进模型的第一层**：模型实际接收的是原始 uint8 像素，内部做 $x'=(x-\text{mean})/\text{std}$。

- 若训练时的预处理是"像素 / 255"，则 `mean=0, std=255`：像素 128 → $128/255=0.502$。
- 这意味着**板端喂给模型的是 0–255 的原始像素**，Python 里不要再除以 255，否则相当于除了两次（输入 $\approx0.002$，几乎全黑，检测全无）。
- 反之，如果转换时没设 mean/std 而 Python 里也没归一化，输入是 0–255 的大数，模型当然失效。
- 你的转换配置里是 `mean=0, std=255`（你学习笔记中记录的源码配置）。口头可以说："the normalization (divide by 255) is folded into the RKNN model, so on the board I feed raw RGB uint8 pixels."

### 9.4 校准集清单

`dataset.txt` 每行一个图片路径。工具链会按同样的预处理（mean/std）对这些图做前向统计。**图片尺寸要和模型输入一致或由你在喂入前做 letterbox**（具体行为看版本）。

### 9.5 精度分析工具

`accuracy_analysis` 一类功能会输出每层量化模型相对浮点的**余弦距离/欧氏距离**，帮助定位掉点的层。典型做法：逐层看，哪一层的余弦相似度突然下降（例如从 0.999 降到 0.9），就把那一层改为 FP16（混合量化）或调整校准。

---

## 10. 预处理：mean/std 与 letterbox

### 10.1 完整预处理链

| 步骤 | 目的 | 常见错误 |
|---|---|---|
| 相机原始帧（如 YUYV/MJPEG）→ BGR/RGB | 得到三通道图 | 色彩空间转换错 |
| 通道顺序 → 与训练一致（多为 RGB） | OpenCV 读出来是 BGR | 红蓝颠倒，精度下降 |
| letterbox：保比例缩放 + 补边（灰 114） | 避免拉伸变形 | 直接 resize 导致形变 |
| 数据类型 uint8，布局 NHWC（板端常见） | NPU 输入格式 | 布局/类型与模型不符 |
| mean/std | 已写入模型 | 重复归一化或漏归一化 |

### 10.2 letterbox 的双向变换

正向：$\text{scale}=\min(W_t/W_0,H_t/H_0)$；缩放后尺寸 $(W_0\cdot\text{scale},H_0\cdot\text{scale})$；补边 $\text{pad}_x=(W_t-W_0\text{scale})/2$，$\text{pad}_y$ 同理。

逆向（框还原到原图）：
$$x_{orig}=\frac{x_{model}-\text{pad}_x}{\text{scale}},\qquad y_{orig}=\frac{y_{model}-\text{pad}_y}{\text{scale}}$$

教学示例：原图 $1280\times720$，目标 $640\times640$：scale $=\min(0.5,0.889)=0.5$，缩后 $640\times360$，$\text{pad}_y=140$。模型输出 $y=200$ → $(200-140)/0.5=120$。

---

## 11. 后处理在量化空间的优化

### 11.1 问题

INT8 模型的输出张量是**整数**，附带每个输出的 $(s,z)$。YOLOv5 三路输出共约 $25200\times(5+n_c)$ 个元素（见 [YOLO 章](./03_卷积网络与目标检测_YOLO.md)）。朴素做法：先把整个张量反量化成 float，再 sigmoid、阈值过滤。这要对几十万个元素做浮点运算，而 CPU（A76/A55）本来就要承担 NMS 等工作，后处理很容易成为瓶颈。

### 11.2 优化思想

> 阈值过滤只需要"这个元素大不大于阈值"，而量化映射是**单调递增**的，所以可以**把阈值量化到整数域，直接比较整数**；只有通过的少数候选才反量化并解码。

推导：若 $\hat x=s(q-z)$，则 $\hat x>\tau\iff q>z+\tau/s\iff q>q_\tau$，其中 $q_\tau=\mathrm{round}(\tau/s)+z$。

若模型输出是 **sigmoid 之前**的原始值（常见：解码交给 CPU），而你要过滤 $\sigma(t)>\text{thr}$，因为 sigmoid 单调，等价于 $t>\mathrm{logit}(\text{thr})=\ln\frac{\text{thr}}{1-\text{thr}}$，再按上式量化。

### 11.3 手算（教学示例）

$\text{thr}=0.25$：$\mathrm{logit}(0.25)=\ln(0.25/0.75)=-1.0986$。该输出张量 $s=0.06,\ z=-17$：$q_\tau=\mathrm{round}(-1.0986/0.06)+(-17)=\mathrm{round}(-18.31)-17=-18-17=-35$。验证：$q=-35$ 对应 $\hat t=0.06\times(-35+17)=-1.08$。于是对 25200 个 obj 值，用整数比较 `q > -35` 筛选（可向量化、可用 SIMD），例如 25200 个里只剩 20 个通过；仅对这 20 个反量化、做 sigmoid、解码、NMS。

> 若 sigmoid 已经包含在 NPU 的模型图里（输出已是概率），则直接 $q_\tau=\mathrm{round}(\text{thr}/s)+z$。**具体是哪一种取决于你的 ONNX 怎么导出**。

### 11.4 注意

- 每个输出张量的 $(s,z)$ 不同，要分别量化阈值。
- INT8 输出精度有限：分数接近 0 或 1 时分辨率粗，同分的候选多，排序（NMS 前）出现平局，通常影响很小但要知道。
- 其他可优化点：先用 obj 过滤再算 cls；只取 argmax 类别；NMS 前限制候选数上限。

本项目的做法：后处理可以在量化空间做阈值过滤（把阈值量化成 $q_{\text{thr}}$ 后直接比较整数，再对少数通过的候选反量化）；三路输出为 $(1,3(5+n_c),80,80)$、$(1,3(5+n_c),40,40)$、$(1,3(5+n_c),20,20)$，$n_c=5$ 时通道数为 30；输出为 sigmoid 之前的原始值，sigmoid 与解码在 CPU 后处理里完成。

---

## 12. 性能与精度评估

### 12.1 精度：怎么证明量化没毁掉模型

1. **同数据、同指标对比**：在同一个带标注验证集上，分别算 FP32 ONNX（在 PC 上）和 INT8 RKNN 的 precision/recall/mAP。报告"掉了多少"，而不是只报告 INT8 的数字。
2. **板端 vs 模拟器**：确认板端结果与 Toolkit 模拟器一致，排除预处理/布局问题。
3. **逐层余弦相似度**：定位敏感层。
4. **现场样本回放**：用真实相机采集的图，人工检查漏检与误检。
5. 调整手段：更换/扩充校准集、换量化算法（min-max→KL）、关键层混合精度、必要时 QAT。

**精度对比记录模板**（把你的实测填进去，没有的格子留空，不要估）：

| 模型 | 精度类型 | 验证集 | mAP@0.5 | Precision | Recall | 备注 |
|---|---|---|---|---|---|---|
| ONNX（PC） | FP32 | 同一份 | — | — | — | 基准 |
| RKNN（模拟器） | INT8 | 同一份 | — | — | — | 排除板端因素 |
| RKNN（板端） | INT8 | 同一份 | — | — | — | 最终 |

**掉点排查流程**：(1) FP32 ONNX 本身在验证集上是否正常（排除导出问题）；(2) RKNN 模拟器 FP16/不量化是否正常（排除转换问题）；(3) 开启量化后掉点多少；(4) 逐层余弦相似度，定位敏感层；(5) 换校准集/算法；(6) 敏感层混合精度；(7) 仍不行再考虑 QAT。每一步只改一个因素，才能知道哪个因素起作用。

### 12.2 性能：什么叫"快"

- **推理延迟**：NPU 一次前向的时间。
- **端到端延迟**：采集→预处理→推理→后处理→传输/落库。教学示例：采集 10 ms + 预处理 5 + 推理 20 + 后处理 5 + 传输 10 = 50 ms。只看推理 20 ms 得到"50 FPS"，端到端却是 20 次/s（1000/50）。
- **吞吐 vs 延迟**：流水线并行时，吞吐可以接近 $1/\max(\text{各级耗时})$，但单帧延迟仍是总和。必须明确定义说的是哪一个。
- **其他指标**：功耗、温度与降频、内存占用、长时间运行稳定性。
- **测量方法**：预热若干次后取多次平均；固定输入；记录时钟与频率设置；说明是否包含内存拷贝。

**诚实原则**：延迟、FPS、mAP 变化要写成测量方法加量级，不要报没有来源的精确数字。
本项目的设计取值与目标（量级，调参时按实机微调）：模型输入 640×640；RK3588 NPU 上单帧模型推理为十几到几十毫秒量级，端到端（含取帧、预处理、后处理）目标 10–20 fps；INT8 相对 FP32 的精度下降目标控制在 mAP 1–3 个点以内；校准集取 100–300 张量级，来自真实部署场景的叶片图像。

---

## 13. 在你的项目里怎么用

对应 CV：**"built a YOLOv5/RKNN INT8 inference chain on RK3588, converting camera frames into structured observations with confidence and timestamps."**

| 链路环节 | 本章知识 |
|---|---|
| 相机帧 → 预处理 | letterbox、通道顺序、uint8/NHWC、mean/std 已折叠进模型 |
| RKNN INT8 推理 | 量化原理、校准集代表性、版本匹配 |
| 后处理 | 量化空间阈值过滤、解码、NMS、坐标还原 |
| 结构化观测（confidence + timestamp） | confidence 是模型分数；时间戳要在统一时钟源上打 |
| 上层 Agent 决策 | 空检测≠健康；低置信度作弱证据，见[Agent 章](./05_Transformer_LLM与Agent_LangGraph.md) |

项目入口：[P4 OpenHarmony 智能园艺系统](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)。平台背景：[RK3588 平台基础](../08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)。

诚实表述模板：

> I focused on the deployment side. I took a trained YOLOv5 model, exported it to ONNX, converted it to an INT8 RKNN model with a calibration set, and built the board-side chain: preprocessing, NPU inference, post-processing and structured output.

（只在你确实如此时使用；若模型是你训练的，再加一句训练经历。）

---

## 14. 常见错误与故障现象

| 现象 | 原因 | 排查 |
|---|---|---|
| 板端几乎检测不到任何东西 | mean/std 重复或遗漏；通道顺序错 | 对照 config 与 Python 预处理，打印输入张量范围 |
| 框位置整体偏 | letterbox 逆变换漏掉补边 | 手算一个框验证 |
| INT8 精度大幅下降 | 校准集不代表现场；离群值撑大范围；敏感层 | 换校准集；逐层余弦分析；混合精度 |
| PC 模拟器结果对、板端错 | 板端预处理/布局不同；运行时版本不匹配 | 固定同一输入逐步对比 |
| 模型加载失败 | 工具链与运行时版本不匹配；平台参数错 | 对齐版本、`target_platform` |
| 后处理很慢 | 反量化整个张量、候选未先过滤 | 量化空间阈值过滤 |
| 帧率达不到预期 | 把峰值 TOPS 当实测；CPU 后处理/拷贝成瓶颈 | 分段计时 |
| 结果偶有时间错位 | 时间戳在不同阶段取自不同时钟 | 统一在采集时打时间戳 |

---

## 15. 面试问答（英文）

**Q1. Why did you quantize the model, and what are the trade-offs?**

> The RK3588's NPU is built for INT8, so quantization reduces model size and memory bandwidth and increases throughput at lower power. The trade-off is accuracy: rounding error and clipping error. I manage it with a representative calibration set and by comparing mAP between the FP32 and INT8 models on the same validation data.

中文要点：NPU 为 INT8 设计；体积/带宽/速度/功耗；代价是舍入与饱和误差；用校准集和同数据对比控制。
追问：*What if accuracy drops too much?* — 换校准集或算法，逐层余弦定位敏感层并混合精度，仍不行则 QAT。

**Q2. Explain the quantization formula.**

> q equals round of x over s plus z, clipped to the integer range, and the dequantized value is s times q minus z. Scale s is the step size, and zero point z is the integer that represents real zero. Rounding error is at most half a scale, but values outside the range are clipped, and that error can be large.

**Q3. Per-tensor versus per-channel quantization?**

> Per-tensor uses one scale for the whole weight tensor, so channels with small weights lose resolution. Per-channel gives each output channel its own scale, which is more accurate for convolution weights. Activations are usually quantized per tensor because per-channel activation scales would break the integer accumulation structure.

**Q4. PTQ versus QAT?**

> Post-training quantization calibrates a trained model with a small unlabeled dataset; it is quick and usually enough for 8 bit. Quantization-aware training inserts fake-quantization into training and uses a straight-through estimator so the network adapts to the quantization error; it costs more but recovers accuracy when PTQ is not enough.

**Q5. How did you optimise post-processing on the board?**（按你的实际情况调整）

> Instead of dequantizing the whole output tensor, I quantize the confidence threshold into the integer domain and filter by integer comparison, because the quantization mapping is monotonic. Only the few surviving candidates are dequantized and decoded, which saves CPU time.

**Q6. After INT8 conversion the detections look much worse. How do you debug?**

> I change one factor at a time. First I confirm the FP32 ONNX model works on the validation set, then that the converted model without quantization works, which isolates export and preprocessing errors. Then I enable INT8 and measure the accuracy drop. If it drops, I compare layer-by-layer cosine similarity to find sensitive layers, try a better calibration set or algorithm, and keep those layers in higher precision if needed. Only if that fails would I consider quantization-aware training.

**Q7. Why does the zero point matter, and why must real zero be exactly representable?**

> Zero padding in convolutions and zeros produced by ReLU appear everywhere. If real zero mapped to a non-integer, every padded pixel or ReLU output would carry a systematic error that accumulates. The zero point guarantees that real zero maps exactly to an integer.

**Q8. Is conversion the same as training?**

> No. Training learns weights from labelled data using a loss. Conversion takes existing weights, changes the format and number representation for the device, and needs only a calibration set. 【我在项目中负责的是……请据实补充】

---

## 16. 自测题

1. $s=0.05$，$z=0$，INT8，$x=1.23$ 与 $x=8.0$ 的量化结果和误差？
   **答案**：$1.23/0.05=24.6\to25\to1.25$，误差 $+0.02$；$8.0/0.05=160\to$ clip 127 $\to6.35$，误差 $-1.65$。
2. 范围 $[0,6]$ 的激活，UINT8 非对称的 $s,z$？对称 INT8 的 $s$？
   **答案**：$s=6/255=0.02353$，$z=0$；对称 INT8 取 $\alpha=6$，$s=6/127=0.0472$（正半边只用 127 级）。
3. 为什么 INT8 乘加要用 INT32 累加？
   **答案**：单个乘积最大 16129，多项累加会超过 INT16/INT8 范围，必须用 INT32 避免溢出。
4. 阈值 $\text{thr}=0.5$，输出为 sigmoid 前、$s=0.1,z=0$，求整数阈值。
   **答案**：$\mathrm{logit}(0.5)=0$，$q_\tau=0$，即 $q>0$。
5. mean=0、std=255 已写入 RKNN，Python 里又除以了 255，会怎样？
   **答案**：输入变成约 $x/255^2$，数值接近 0，模型接近"全黑图"，几乎检测不到目标。
6. 两个输出通道范围 $[-1,1]$ 与 $[-4,4]$，通道 1 的权重 0.3，逐层与逐通道的量化值各是多少？
   **答案**：逐层 $s=4/127=0.0315$，$q=\mathrm{round}(9.525)=10$（$\hat w=0.315$）；逐通道 $s=1/127=0.00787$，$q=\mathrm{round}(38.1)=38$（$\hat w=0.2992$）。逐通道更准。
7. 推理 20 ms，端到端 50 ms，能说"50 FPS"吗？
   **答案**：不能。50 FPS 只对应推理一项；端到端按 50 ms 为 20 次/s（流水线并行时吞吐可更高，但需明确定义）。
8. 非对称量化，范围 $[-2,6]$，UINT8，求 $s,z$，以及 $x=0$ 与 $x=1$ 的量化值。
   **答案**：$s=8/255=0.03137$；$z=\mathrm{round}(2/0.03137)=\mathrm{round}(63.75)=64$；$x=0\to q=64$；$x=1\to\mathrm{round}(31.875)+64=32+64=96$，反量化 $0.03137\times32=1.004$。
9. 用整数阈值过滤：输出 $s=0.05,z=-10$（INT8），阈值 logit $=-2.0$，整数阈值是多少？
   **答案**：$q_\tau=\mathrm{round}(-2.0/0.05)+(-10)=-40-10=-50$，保留 $q>-50$。
10. 为什么"推理 20 ms"不能直接推出系统每秒处理 50 帧？
   **答案**：还有采集、预处理、后处理和传输；端到端延迟是各段之和，吞吐只在流水线并行且各段不成为瓶颈时才接近 $1/\max$。

---

## 本章要点速记

1. 量化 $q=\mathrm{clip}(\mathrm{round}(x/s)+z)$，反量化 $\hat x=s(q-z)$；舍入误差 $\le s/2$，饱和误差无上界。
2. 对称 $z=0$ 硬件简单；非对称用满 256 级，适合非负激活；浮点 0 必须被精确表示。
3. 权重逐通道、激活逐层是常见组合。
4. 校准集必须代表部署数据，预处理要与部署完全一致；离群值会撑大 min-max 范围。
5. INT8 乘加用 INT32 累加，再乘 $s_ws_x$ 还原或重量化到下一层。
6. PTQ 快、无标签；QAT 用伪量化+STE，精度更好但成本高；敏感层可 FP16 混合精度。
7. RKNN 流程：ONNX → config(mean/std/target) → build(量化+校准集) → export → 板端 runtime 推理；版本必须匹配。
8. mean/std 折叠进模型，板端喂原始 uint8，不可重复归一化。
9. 量化映射单调，阈值可量化后在整数域比较，只对存活候选反量化解码。
10. 精度看同数据 FP32 vs INT8，逐层余弦定位；性能区分推理延迟、端到端延迟、吞吐；没有实测就不报数字。

## 相关链接

- 上一章：[03 卷积网络与目标检测 YOLO](./03_卷积网络与目标检测_YOLO.md)
- 下一章：[05 Transformer、LLM 与 Agent、LangGraph](./05_Transformer_LLM与Agent_LangGraph.md)
- 相关项目：[P4 OpenHarmony 智能园艺系统](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)
- 平台背景：[OpenHarmony / Hi3861 / RK3588 平台基础](../08_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)
- 八股速查：[模型部署与量化八股](../../03_八股知识/02_人工智能/06_模型部署与量化八股.md)
