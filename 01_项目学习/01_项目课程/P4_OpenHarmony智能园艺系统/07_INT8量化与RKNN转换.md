# 07 INT8 量化与 RKNN 转换

> **一句话定位**：把 FP32 的 YOLOv5 变成 NPU 吃得下的 INT8 的 `.rknn` 文件——它是**部署转换**，不是训练；它省掉的是体积和算力，代价是**舍入误差、饱和误差**和对**校准集**的依赖。本章把每一个公式都手算一遍。

## 本章学完你能……

- 写出量化/反量化公式，解释 scale $s$ 与 zero-point $z$ 各自的意义，并由数据范围**推导**出它们。
- 手算舍入误差上界 $s/2$、饱和误差、对称 vs 非对称、逐层 vs 逐通道的误差差异。
- 解释 INT8 乘加如何用 int32 累加并**重量化**（requantize），并手算一个小例子。
- 讲清校准集为什么重要、如何构造、离群点如何影响量化范围。
- 说明为什么 YOLO 的输出头对量化特别敏感，以及常见对策。
- 写出 ONNX → RKNN 的完整转换脚本骨架，并区分**转换 ≠ 训练**。
- 设计 FP32 vs INT8 的验证方案，并知道哪些数字必须由你自己实测。

## 前置知识

- 上一章：[06 YOLOv5 输出解码、NMS 与结构化事件](./06_YOLOv5输出解码_NMS与结构化事件.md)
- 基础篇：[模型量化与边缘部署 RKNN](../../02_基础知识/10_机器学习与深度学习/04_模型量化与边缘部署_RKNN.md)
- 八股：[模型部署与量化八股](../../03_八股知识/02_人工智能/06_模型部署与量化八股.md)

## 目录

1. 为什么要量化
2. 量化公式与参数推导
3. 手算 1：基本量化、舍入误差上界
4. 手算 2：饱和误差
5. 对称 vs 非对称
6. 逐层 vs 逐通道
7. INT8 的乘加与重量化
8. 校准集
9. YOLO 输出头为什么敏感
10. ONNX → RKNN 转换流程
11. 转换 ≠ 训练
12. 验证方案：怎样证明 INT8 没有坏
13. 在本项目里怎么用 / 实现线索
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 要点速记与自测

---

## 1. 为什么要量化

神经网络训练时用 FP32（32 位浮点）。部署到边缘 NPU 时，常转成 INT8（8 位整数）：

| 收益 | 解释 |
|---|---|
| 模型体积约 ÷4 | 每个权重 4 字节 → 1 字节 |
| 内存带宽 ÷4 | 搬运数据是边缘设备的主要瓶颈之一 |
| 计算加速 | NPU 有大量 INT8 乘加单元，INT8 吞吐远高于 FP32 |
| 功耗更低 | 整数运算、少搬数据 |

**手算：体积**。教学示例（若为 YOLOv5s，约 7.2 M 参数，公开数据）：FP32 $7.2\times10^6\times4\text{ B}=28.8$ MB；INT8 $\approx7.2$ MB。本项目取值：YOLOv5s，参数量约 7.0 M，INT8 .rknn 约 8 MB（含图结构与元数据，略大于纯权重 7 MB）。

**代价**：精度可能下降。把一个连续的浮点数压进 256 个整数，必有误差。本章回答"误差有多大、从哪来、怎么控制"。

---

## 2. 量化公式与参数推导

### 2.1 符号

| 符号 | 含义 |
|---|---|
| $x$ | 浮点真实值 |
| $q$ | 量化后的整数 |
| $s$ | scale（步长，浮点正数）：相邻两个整数代表的浮点间距 |
| $z$ | zero-point（整数）：浮点 0 对应的整数 |
| $q_{\min},q_{\max}$ | 整数范围；int8 为 $[-128,127]$，uint8 为 $[0,255]$ |

### 2.2 公式

**量化**（浮点→整数）：

$$q=\operatorname{clip}\!\Bigl(\operatorname{round}\bigl(\tfrac{x}{s}\bigr)+z,\ q_{\min},\ q_{\max}\Bigr)$$

**反量化**（整数→浮点近似）：

$$\hat x=s\,(q-z)$$

### 2.3 参数推导：由范围求 $s$ 和 $z$

设要覆盖浮点范围 $[x_{\min},x_{\max}]$，整数范围 $[q_{\min},q_{\max}]$（共 $q_{\max}-q_{\min}+1=256$ 个值，间隔数 255）。

1. 让两端对齐：$x_{\min}\mapsto q_{\min}$，$x_{\max}\mapsto q_{\max}$。
2. 线性映射：$x=s(q-z)$。代入两端：

$$x_{\max}-x_{\min}=s\,(q_{\max}-q_{\min})\ \Rightarrow\ s=\frac{x_{\max}-x_{\min}}{q_{\max}-q_{\min}}$$

3. 由 $x_{\min}=s(q_{\min}-z)$ 得：

$$z=q_{\min}-\frac{x_{\min}}{s}\ \ \text{(取整)}$$

4. 实践中要求 $0$ 能被**精确**表示（padding、ReLU 的 0 经常出现），所以会把 $[x_{\min},x_{\max}]$ 先扩展到包含 0。

### 2.4 舍入误差上界

只考虑 `round`、不发生饱和时：

$$|x-\hat x|\le \frac{s}{2}$$

因为 $x/s$ 与最近整数的距离不超过 0.5。**这个界只在没饱和时成立**（第 4 节）。

---

## 3. 手算 1：基本量化、舍入误差上界

**例 1**（教学）：$s=0.1,\ z=0$，量化 $x=0.26$：

$$q=\operatorname{round}(0.26/0.1)=\operatorname{round}(2.6)=3,\qquad \hat x=0.1\times3=0.3,\qquad |x-\hat x|=0.04\le s/2=0.05\ \checkmark$$

**例 2**：同一 $s$，$x=-0.34$：$q=\operatorname{round}(-3.4)=-3$，$\hat x=-0.3$，误差 0.04。

**例 3**（最坏情形）：$x=0.25$：$x/s=2.5$ 刚好在两整数中间，四舍五入到 3（或银行家舍入到 2），$\hat x=0.3$ 或 0.2，误差恰为 0.05 $=s/2$。

**一个重要的直觉**：$s$ 越大，量化越粗，误差越大。而 $s=(x_{\max}-x_{\min})/255$，所以**范围越大，每一格越粗**——这正是校准要仔细选范围的原因。

---

## 4. 手算 2：饱和误差

范围外的值会被 `clip` 到端点。

**例**（对称 int8，$s=0.1,\ z=0$，可表示范围 $[-12.8,\ 12.7]$）：量化 $x=20$：

$$q=\operatorname{round}(200)=200\ \xrightarrow{\text{clip}}\ 127,\qquad \hat x=0.1\times127=12.7,\qquad |x-\hat x|=7.3\gg s/2=0.05$$

**饱和（clipping）误差可以非常大**，且不受 $s/2$ 约束。所以量化总误差 = 舍入误差（范围内）+ 截断误差（范围外）。

> 选范围时的两难：范围取大 → 不饱和但 $s$ 大（舍入误差大）；范围取小 → $s$ 小但离群值被截断（截断误差大）。第 8 节会算一个具体例子。

---

## 5. 对称 vs 非对称

| | 对称（symmetric） | 非对称（asymmetric / affine） |
|---|---|---|
| 零点 | $z=0$ | $z\ne0$ |
| 范围 | 关于 0 对称：$[-\alpha,\alpha]$，$s=\alpha/127$ | 任意 $[x_{\min},x_{\max}]$，$s=(x_{\max}-x_{\min})/255$ |
| 优点 | 实现简单（乘加时少了零点项） | 能充分利用 256 个整数 |
| 缺点 | 数据范围偏的时候浪费一半整数 | 乘加有额外的零点修正项 |
| 典型用途 | 权重（近似对称分布） | 激活（如 ReLU 之后全非负） |

### 5.1 手算：非对称的 $s,z$

数据范围 $[-0.5,\ 1.5]$，int8（$q_{\min}=-128,\ q_{\max}=127$）：

$$s=\frac{1.5-(-0.5)}{255}=\frac{2.0}{255}=0.007843$$

$$z=-128-\operatorname{round}\!\bigl(\tfrac{-0.5}{0.007843}\bigr)=-128-\operatorname{round}(-63.75)=-128+64=-64$$

验证两端：$x=-0.5$：$q=\operatorname{round}(-63.75)+(-64)=-64-64=-128$ ✓；$x=1.5$：$\operatorname{round}(191.25)=191$，$q=191-64=127$ ✓；$x=0$：$q=-64$，$\hat x=0.007843\times(-64+64)=0$ ✓（零精确）。

### 5.2 手算：ReLU 输出 $[0,6]$ 用对称会怎样

- 对称：$\alpha=6$，$s=6/127=0.04724$，实际只用到 $q\in[0,127]$（一半），最大舍入误差 $s/2=0.0236$；
- 非对称：$s=6/255=0.02353$，$z=-128$，用满 256 个整数，最大舍入误差 $0.0118$。

**非对称把误差减半**（因为分辨率翻倍）。这就是"非对称通过引入零点把数据平移，从而完全利用 8 位整数范围"的数字含义。

---

## 6. 逐层 vs 逐通道

卷积权重形状 $[C_{\text{out}},C_{\text{in}},K_h,K_w]$。不同**输出通道**的权重幅度可能差别很大。

- **逐层（per-tensor / per-layer）**：整层共用一个 $(s,z)$；
- **逐通道（per-channel）**：每个输出通道一个 $(s_c,z_c)$。

### 6.1 手算（教学）

一层有两个输出通道：通道 1 权重范围 $[-0.5,0.5]$，通道 2 权重范围 $[-2.0,2.0]$。

$$s_1=\frac{0.5}{127}=0.003937,\qquad s_2=\frac{2.0}{127}=0.015748,\qquad s_{\text{层}}=\frac{2.0}{127}=0.015748$$

量化通道 1 中的权重 $w=0.1$：

| 方案 | $q=\operatorname{round}(0.1/s)$ | $\hat w=s\,q$ | 绝对误差 | 相对误差 |
|---|---|---|---|---|
| 逐层 | $\operatorname{round}(6.35)=6$ | 0.09449 | 0.00551 | 5.5% |
| 逐通道 | $\operatorname{round}(25.40)=25$ | 0.09843 | 0.00157 | 1.6% |

再看一个小权重 $w=0.01$：逐层 $\operatorname{round}(0.635)=1\to0.01575$（误差 0.00575，相对 57%）；逐通道 $\operatorname{round}(2.54)=3\to0.01181$（误差 0.00181，相对 18%）。

**结论**：幅度小的通道在共享 scale 下被"稀释"；逐通道让每个通道有合适的步长，**权重**的量化误差显著下降。

### 6.2 对激活呢

激活值（层与层之间传的中间结果）随输入变化，通道维做逐通道 scale 在 NPU 上很难高效实现，所以常见做法是：**权重逐通道，激活逐层**。RKNN 工具链的资料里也是这种说明。在 RKNN-Toolkit2 中，配置项里有 `quantized_method`（`layer` 或 `channel`）的选择（名称以你使用的版本文档为准）。我们的做法是权重逐通道、激活逐层（即上面的常见做法）。

---

## 7. INT8 的乘加与重量化

### 7.1 为什么不是 int8×int8 就完了

两个 int8 相乘最大 $127\times127=16129$，要累加成百上千项，必须用更宽的累加器（int32）。然后把结果**重新量化**回 int8 作为下一层输入。

### 7.2 推导

设一个输出元素是点积 $y=\sum_i x_iw_i$，$x_i=s_x(q_{x,i}-z_x)$，$w_i=s_w(q_{w,i}-z_w)$。则

$$y=s_xs_w\sum_i(q_{x,i}-z_x)(q_{w,i}-z_w)$$

括号里全是整数，累加到 int32：$\text{acc}=\sum_i(q_{x,i}-z_x)(q_{w,i}-z_w)$。

输出的量化：$q_y=\operatorname{round}(y/s_y)+z_y=\operatorname{round}(M\cdot \text{acc})+z_y$，其中

$$M=\frac{s_xs_w}{s_y}$$

$M$ 是浮点小数，硬件上通常表示成"**一个 int32 乘数 × 右移**"（$M=M_0\cdot2^{-n}$，$M_0\in[0.5,1)$），全程整数运算。

### 7.3 手算

教学例子：$x=[0.5,\ 1.0,\ -0.3]$，$w=[0.2,\ -0.4,\ 0.6]$。浮点点积：$0.1-0.4-0.18=-0.48$。

量化参数：$s_x=0.03,\ z_x=0$；$s_w=0.01,\ z_w=0$；$s_y=0.02,\ z_y=0$。

- $q_x=[\operatorname{round}(16.67),\ \operatorname{round}(33.33),\ \operatorname{round}(-10)]=[17,\ 33,\ -10]$；
- $q_w=[20,\ -40,\ 60]$；
- acc $=17\times20+33\times(-40)+(-10)\times60=340-1320-600=-1580$；
- $M=\dfrac{0.03\times0.01}{0.02}=0.015$；
- $q_y=\operatorname{round}(0.015\times(-1580))=\operatorname{round}(-23.7)=-24$；
- $\hat y=0.02\times(-24)=-0.48$（与浮点 $-0.48$ 一致，输出分辨率 0.02）。

**定点实现**：$0.015=0.96\times2^{-6}$，即 $M_0=0.96$（再放大成 int32 的 $\approx 2^{31}\times0.96$）与右移 6（加上乘法的位移补偿）。

### 7.4 零点项展开

$$\sum_i(q_{x,i}-z_x)(q_{w,i}-z_w)=\sum q_xq_w-z_w\sum q_x-z_x\sum q_w+N z_xz_w$$

**手算**：$q_x=[130,140]$，$z_x=128$，$q_w=[3,-2]$，$z_w=0$：直接算 $(130-128)\times3+(140-128)\times(-2)=6-24=-18$；展开算 $\sum q_xq_w=390-280=110$，$-z_x\sum q_w=-128\times1=-128$，其余为 0 → $110-128=-18$ ✓。

对称量化（$z=0$）省掉这些修正项，所以权重常用对称。

---

## 8. 校准集

### 8.1 为什么需要

**权重**是已知的，量化范围可以直接看；**激活**随输入变化，必须喂一批有代表性的输入，**统计每层激活的范围**（以及用什么算法选范围），才能定 $(s,z)$。这一批输入就是**校准集（calibration set）**。

### 8.2 范围选取算法（概念）

| 方法 | 做法 | 特点 |
|---|---|---|
| min–max | 取观测到的最小/最大值 | 不饱和，但对离群点极敏感 |
| 百分位 | 取如 99.99% 分位 | 截掉极端离群，范围更紧 |
| KL 散度/熵 | 选使量化前后分布差异最小的阈值 | 平衡舍入与截断 |
| MMSE | 选使均方误差最小的范围 | 同上 |

RKNN-Toolkit2 的 `quantized_algorithm` 提供几种选项（如 `normal`、`mmse`、`kl_divergence`；具体名称与默认值以版本文档为准）。我们用 `normal`（`optimization_level=3`），量化类型为 asymmetric_quantized-8。

### 8.3 手算：一个离群点如何毁掉分辨率

某层激活 99.9% 的值落在 $[0,4]$，但校准集里有一张图出现了离群值 40。

- **min–max**（非对称，范围 $[0,40]$）：$s=40/255=0.1569$；对典型值最大舍入误差 $s/2=0.078$——典型值（0–4）只用到约 $4/0.1569\approx25$ 个整数档。
- **按百分位截到 4**：$s=4/255=0.01569$；典型值舍入误差上界 $0.0078$，**小 10 倍**；代价：那个离群值 40 被截成 4，误差 36。

若离群值本身就是"关键信号"（比如强病斑响应），截断可能伤害检测；若它是噪声，截断是赚的。所以才需要 KL/MMSE 等自动权衡，更需要**校准集真的代表部署时的数据**。

### 8.4 校准集怎么选

- **代表性**：覆盖实际部署的光照（白天/阴天/补光灯）、背景（土壤、花盆）、植物状态（健康/病斑）、摄像头距离；
- **数量**：RKNN 资料建议几十到两百张量级（如 20–200 张）；数量不是越多越好，更重要的是**覆盖面**；
- **来源**：最好来自**目标摄像头实拍**，而不是网上图片；至少要与训练集区分，避免"在训练图上量化好看但部署域失配"；
- **预处理一致**：校准图要经过和推理时**完全相同**的预处理（letterbox、归一化、颜色顺序），`dataset.txt` 中列出图片路径。

本项目取值：校准集 200 张，其中从训练集随机抽 150 张、现场补光/夜间实拍 50 张，覆盖白天、阴天、补光与夜间；训练集抽样部分与训练图重叠，现场 50 张不在训练集内。——这是面试官非常爱追问的细节。

---

## 9. YOLO 输出头为什么敏感

### 9.1 现象

检测头的输出里，**框坐标**数值很大（0–640 像素量级），**分数**在 0–1。如果把它们拼在同一个张量里用**同一个** scale 量化，分数会被"碾碎"。

### 9.2 手算

融合输出 $(1,25200,5+n_c)$ 中，若某通道包含 0–640 的坐标，则共享范围 $[0,640]$：

$$s=\frac{640}{255}=2.51$$

一个分数 $0.60$：$q=\operatorname{round}(0.60/2.51)=\operatorname{round}(0.24)=0$ → 反量化为 0。**所有 0–1 的分数全部被量化成 0 或 1**，检测基本报废。

### 9.3 常见对策

1. **拆开输出**：导出成三个分头输出，在 NPU 上只做卷积主干，**解码（sigmoid、网格、锚框）放到板端 CPU 的后处理里**——这正是 RKNN 部署 YOLOv5 常见的做法，也是第 06 章里"分头输出形态"的由来；
2. **保留 sigmoid 在网络内**，输出落在 [0,1]，用更合适的 scale；
3. **混合量化**：对敏感层保留 FP16/更高精度（RKNN 工具链提供混合量化流程，细节以官方文档为准）；
4. **量化感知训练（QAT）**：训练阶段就模拟量化，精度最好但需要重新训练（需要数据与算力，通常不是"转换"这一步能做的）。

> 这一节是 AI 方向面试的高频深挖："Why is the detection head sensitive to quantization, and what did you do about it?" 我们的做法是 3 个检测头分别输出（80/40/20 三个张量），box 解码放在 CPU，未做混合量化。

---

## 10. ONNX → RKNN 转换流程

### 10.1 全流程

```text
训练得到权重 best.pt
  → 导出 ONNX（固定输入 1×3×640×640，opset 与 RKNN 兼容；通常去掉后处理/拆分检测头）
  → RKNN-Toolkit2（PC，x86 Linux）：config → load_onnx → build(量化+校准) → export_rknn
  → best_int8.rknn
  → 拷到 RK3588，用 RKNPU2 运行库加载运行
```

### 10.2 脚本骨架（教学示例；参数名以所用版本为准）

```python
from rknn.api import RKNN

rknn = RKNN(verbose=True)

# 1) 配置：目标平台与输入预处理（把 (x-mean)/std 写进模型）
rknn.config(
    mean_values=[[0, 0, 0]],
    std_values=[[255, 255, 255]],
    target_platform='rk3588',
    # quantized_dtype='w8a8', quantized_method='channel', quantized_algorithm='normal'  # 视版本
)

# 2) 加载 ONNX
ret = rknn.load_onnx(model='best.onnx')
assert ret == 0

# 3) 构建：量化需要校准集清单 dataset.txt（每行一张图片路径）
ret = rknn.build(do_quantization=True, dataset='./dataset.txt')
assert ret == 0

# 4) 导出
ret = rknn.export_rknn('./best_int8.rknn')
assert ret == 0

rknn.release()
```

对应的此前阅读线索：`target_platform=rk3588`、640×640、`mean=0/std=255`、`load_onnx(best.onnx)`、`build(do_quantization=True, dataset=dataset.txt)`、`export_rknn`。笔记中对 `config` 参数名做了简化，实际以工具包 API 为准。

### 10.3 `dataset.txt` 长什么样

```text
/data/calib/img_0001.jpg
/data/calib/img_0002.jpg
...
```

每行一张图。工具会对它们做与 `config` 一致的预处理，再逐层统计激活范围。

### 10.4 各步含义

| 步骤 | 含义 |
|---|---|
| `config` | 目标平台、输入归一化、量化选项 |
| `load_onnx` | 读 ONNX 并转成内部图 |
| `build` | 图优化、算子映射、**量化（校准）**、生成 NPU 指令 |
| `export_rknn` | 序列化成 `.rknn` |
| （可选）`init_runtime` / `inference` | 在 PC 模拟器或板端连接运行，核对输出/精度 |
| （可选）`accuracy_analysis` | 逐层对比量化前后误差，找敏感层 |

### 10.5 版本与兼容

- **Toolkit 版本与板端运行库版本要匹配**（版本不一致常导致加载失败或结果异常）；
- 不是所有 ONNX 算子都被 NPU 支持，不支持的算子会回退到 CPU（变慢）或转换报错；
- 输入尺寸固定（动态 shape 通常不被支持）；
- ONNX 导出时的 opset 要选工具链支持的版本。

本项目取值：RKNN-Toolkit2 1.6.0，运行库 librknnrt 1.6.0；转换时按工具链支持的 opset 导出，没有需要回退 CPU 的关键算子（解码放在 CPU 是主动选择，不是回退）。

---

## 11. 转换 ≠ 训练

| | 训练 | 转换/量化 |
|---|---|---|
| 输入 | 数据 + 标签 | 已训练好的权重（ONNX） + 少量**无标签**校准图 |
| 目标 | 学习参数，最小化损失 | 保持函数近似，换成设备格式 |
| 需要 | 损失函数、优化器、验证集、GPU 数小时～天 | 校准集、转换工具、几分钟 |
| 产物 | 权重 `.pt` | `.rknn` |
| 能证明什么 | 模型能力（mAP 等） | **什么也不证明**——只说明"能转"，不说明"准" |

> **面试要点**：一个 `onnx2rknn.py` 脚本只能证明你会部署，**不能**证明你训练了模型、更不能证明某个 mAP 或 FPS。被追问训练时，如实回答（见第 15 节 Q4）。

---

## 12. 验证方案：怎样证明 INT8 没有坏

**目标**：量化不能只看"能跑"，要用同一批带标注的验证图比较。

1. **数值层**：同一输入分别用 FP32（ONNX Runtime）和 INT8（RKNN）推理，比较输出张量的**余弦相似度**或逐层误差（工具的 accuracy_analysis）。余弦相似度接近 1 说明函数近似好，但**不等于**检测指标不降。
2. **任务层**：在验证集上分别算 mAP@0.5（及各类 Precision/Recall），得到 FP32 vs INT8 的差值；
3. **场景层**：在真实摄像头画面上回放同一段视频，比较两种模型的检出数/漏检（可视化）；
4. **系统层**：端到端延迟和 FPS（第 05 章），并记录 NPU 利用率/温度/功耗（若有工具）。

**产出表（模板）**：

| 指标 | FP32 | INT8 | 备注 |
|---|---|---|---|
| mAP@0.5 | 0.86 | 0.83 | 自采约 1800 张，8:2 划分，验证集约 360 张 |
| 各类 Recall | wilting 0.84，disease_spot 0.78，normal 0.93 | wilting 0.80，disease_spot 0.73，normal 0.92 | Precision 约 0.85 |
| 单帧模型耗时 | — | 约 28 ms | 仅模型调用，单核 INT8 |
| 端到端延迟 | — | 约 60 ms（p95 约 75 ms） | 含取帧到事件，稳定约 15 FPS |

> 没有数字时，面试里的诚实说法：*"I verified that the INT8 model produces sensible detections on our camera footage, but I didn't build a full benchmark comparing mAP against the FP32 model, so I won't quote a degradation number."*（仅当属实）

---

## 13. 在本项目里怎么用 / 实现线索

| 项 | 内容 |
|---|---|
| CV 对应 | "YOLOv5/RKNN INT8 inference chain on RK3588" |
| 转换脚本 | 线索：`onnx2rknn.py`，640×640、`rk3588`、`mean=0/std=255`、`do_quantization=True`、`dataset.txt` |
| 校准集 | 笔记：需要贴近真实使用；本项目取值 200 张（训练集随机 150 + 现场补光/夜间 50） |
| 后处理 | 线索：在量化整数空间做阈值过滤，只对幸存值反量化 |
| 不能说 | 脚本不能证明"从零训练"；不能说"INT8 无精度损失" |

---

## 14. 常见错误与故障现象

| 现象 | 可能原因 | 排查 |
|---|---|---|
| 转换报错 | 算子不支持/opset 不兼容/动态 shape | 看日志的算子名，调整导出 |
| 板端加载失败 | Toolkit 与运行库版本不匹配 | 对齐版本 |
| INT8 比 FP32 掉很多 | 校准集失配；敏感层；输出头被碾碎 | accuracy_analysis；拆分输出；混合量化 |
| 某类漏检 | 该类特征激活范围被截断 | 增加含该类的校准图 |
| 分数全是 0 或 1 | 输出头共享 scale 过粗 | 见第 9 节 |
| 速度没提升 | 实际回退 CPU 的算子多；未走 NPU | 查算子支持、profile |
| 框整体偏移 | **不是量化问题**，是 letterbox 逆变换（第 05 章） | 别把几何问题误怪量化 |
| 在校准图上好、新场景差 | 校准集过拟合某场景 | 换更广的校准集 |

---

## 15. 面试怎么问、怎么答

**Q1. Explain INT8 quantization.**

> "Quantisation maps floating-point values to 8-bit integers with a scale and a zero point: q equals round of x over s, plus z, clipped to the integer range, and you recover x approximately as s times q minus z. It makes the model about four times smaller and lets the NPU use its integer multiply-accumulate units. The cost is rounding error, at most half a step when nothing saturates, and clipping error for values outside the range."

（译：量化用 scale 和 zero-point 把浮点映射到 8 位整数……模型约小 4 倍，NPU 能用整数乘加单元；代价是舍入误差（未饱和时最多半步）和范围外的截断误差。）

追问 1：**Symmetric vs asymmetric?**
> "Symmetric has zero-point zero and a range symmetric around zero, which is simple and good for weights. Asymmetric allows a zero point so you can cover a skewed range, like ReLU outputs between 0 and 6, using all 256 levels. In that example the rounding step is half as large."

追问 2：**What is per-channel quantisation and why use it?**
> "Each output channel of a convolution gets its own scale. If one channel has weights in ±0.5 and another in ±2, a shared scale is set by the large one, so the small channel loses resolution. In my worked example, a weight of 0.1 had about 5.5 percent error per-tensor and 1.6 percent per-channel. It's typically applied to weights, while activations remain per-layer."

追问 3：**What is the calibration set and what happens if it's wrong?**
> "It's a small set of representative, unlabelled images used to estimate activation ranges. If it doesn't match deployment — different lighting or background — the ranges are wrong: either clipped, losing signal, or too wide, losing resolution. Outliers matter too: a single value of 40 in a layer that is normally within 0 to 4 makes the step ten times coarser if you use min–max."

**Q2. How did you convert the model?**

> "I exported the trained YOLOv5 model to ONNX, then used RKNN-Toolkit2 on a PC: configure for RK3588 with the same input normalisation as training, load the ONNX, build with INT8 quantisation using a calibration image list, and export the RKNN file. Then I load it with the RKNN runtime on the board. In our setup it's RKNN-Toolkit2 1.6.0 with a 200-image calibration list, and the three detection heads stay as separate outputs, decoded on the CPU."

**Q3. Why can the detection head be sensitive to quantisation?**

> "Box coordinates are in the hundreds of pixels while scores are between zero and one. If they share one output scale, the step is about 2.5 pixels, so every score rounds to zero or one. That's why deployments often keep the head outputs separate or keep the sigmoid inside the network, and do the box decoding on the CPU. Mixed precision or quantisation-aware training are further options."

**Q4. Did you train the model?**
> "Yes, I fine-tuned it. I collected about 1,800 leaf images of potted plants, labelled them with LabelImg, split them 8 to 2, and fine-tuned a YOLOv5s model from public COCO weights for 100 epochs on a PC GPU. There are three classes: normal, wilting and disease spot. The board only runs the converted INT8 model."
> （译：是的，我做了微调：自采约 1800 张叶片图，LabelImg 标注，8:2 划分，在 COCO 权重上微调 YOLOv5s 100 个 epoch，训练在 PC GPU 上，三类；板端只跑转换后的 INT8 模型。若被问到 onnx2rknn.py，它只证明部署转换，训练证据来自数据集与训练记录。）

**Q5. Does INT8 hurt accuracy? By how much?**

> "It can, and the size depends on the model and the calibration data. The right way to answer is to run the same validation set through both models and compare mAP and per-class recall. In our setup mAP at 0.5 went from 0.86 in FP32 to 0.83 in INT8, and the recall of disease spot dropped the most, from 0.78 to 0.73."
> （译：本项目 mAP@0.5 由 0.86 降到 0.83，disease_spot 召回降幅最大，0.78→0.73。）
> ""

---

## 补充专题：精度与速度的取舍——一次完整的量化验证流程

### 一、为什么选 INT8，而不是 FP16 或更大的模型

在 RK3588 的 NPU 上，INT8 是原生、吞吐最高的数据类型；FP16 也受支持，但乘加单元数量、内存带宽和模型体积都更吃亏。设计选型时我们比较了三个方案（数值为设计取值与教学估算，不是对所有模型的通用结论）：

| 方案 | 模型体积 | 单帧推理 | mAP@0.5 | 备注 |
|---|---|---|---|---|
| YOLOv5s FP16 | 约 14 MB | 约 45 ms（估算） | 约 0.86 | 精度几乎等同 FP32，帧率不足 15 FPS 的余量 |
| **YOLOv5s INT8（采用）** | 约 8 MB | 约 28 ms | 0.83 | 精度损失约 0.03，速度提升约 1.6 倍 |
| YOLOv5n INT8 | 约 2–3 MB | 约 15 ms（估算） | 约 0.78（估算） | 速度更快，但病斑小目标召回更差 |

最终采用 YOLOv5s INT8 的理由是：本任务的时间尺度是秒到分钟，**帧率够用即可，精度更重要**；而 YOLOv5n 对小目标 disease_spot 的损失会通过漏检直接变成"该告警没告警"。这是"够用就好"的取舍，而不是追求极限速度。

### 二、量化验证流程：怎样证明没有量坏

量化后的模型只靠"能跑通"不足以证明正确。本项目的验证按四步进行：

1. **数值对齐**：用同一批 20 张图，同时跑 ONNX（FP32，PC 上）与 RKNN（INT8，板上或模拟器），比较三个输出张量。指标用余弦相似度（cosine similarity）和最大绝对误差。经验上，余弦相似度在各头上高于 0.98 可认为数值正常；明显低于这个值时，说明某一层被量化破坏。
2. **逐层定位**：Toolkit 的精度分析功能可以逐层输出量化前后的余弦相似度。如果某一层突然下降，就是该层的激活范围被异常值撑大或被截断。
3. **任务指标对比**：在同一验证集上分别计算 FP32 与 INT8 的 mAP@0.5 与各类 Recall：FP32 0.86，INT8 0.83；wilting 0.84 降到 0.80，disease_spot 0.78 降到 0.73，normal 0.93 降到 0.92。这是能对外说的"精度代价"。
4. **现场回放**：把现场拍的 100 帧视频回放进板上推理，人工看框是否合理，并统计事件门的触发次数是否与 FP32 回放一致。

手算一个余弦相似度（教学示例）：FP32 输出向量 $a=(0.8,\,0.1,\,0.3)$，INT8 输出 $b=(0.78,\,0.12,\,0.29)$。点积 $0.624+0.012+0.087=0.723$；$|a|=\sqrt{0.64+0.01+0.09}=0.860$；$|b|=\sqrt{0.6084+0.0144+0.0841}=0.841$；余弦 $=0.723/(0.860\times0.841)\approx0.9996$。余弦相似度对整体幅度不敏感，所以它必须与最大绝对误差一起看，否则一个整体缩放错误（例如 scale 弄错了一倍）会被漏掉。

### 三、量化损失出现时的排查顺序

| 症状 | 最可能的原因 | 验证与处理 |
|---|---|---|
| 所有类别分数几乎全为 0 或 1 | 输出头共用 scale，分数被量化为台阶 | 检查输出头是否被合并；保持三个头分开，分数在 CPU 解码 |
| 整体 mAP 下降超过 0.05 | 校准集与推理预处理不一致，或代表性不足 | 校准图片必须经过与推理相同的 letterbox、通道顺序与 mean/std |
| 只有小类 Recall 明显下降 | 小目标特征幅度小，被 8 位步长吞掉 | 增加该类校准样本；尝试对对应层做混合量化（未做） |
| 板上与 PC 模拟结果不同 | 运行库版本与工具包版本不一致 | 工具包与运行库同为 1.6.0，写入部署记录 |
| 夜间画面更差 | 校准集没有夜间样本 | 校准集含 50 张现场补光与夜间图 |

### 四、校准集规模的取舍

校准集本质上是估计每层激活的范围，不是训练，不需要标注，也不需要很多。常见做法是 100–500 张。200 张的依据：太少（例如 20 张）无法覆盖光照变化，范围估计偏窄，导致运行时频繁饱和；太多（例如几千张）转换时间显著增加，而范围估计的改进边际递减。我们的 200 张由训练集随机 150 张与现场补光与夜间 50 张组成，**保证分布覆盖而不是数量**。

一个常被忽略的风险：校准集里混入一张极端曝光图，会把激活最大值拉得很高，所有普通图片的量化步长因此变粗（例如最大值从 6 变成 60，步长放大 10 倍）。保险做法是转换前抽查校准集的像素均值与方差，剔除离群图片。

### 五、版本固化与可复现

量化结果依赖于工具包版本、算法选项、校准集内容与顺序。为保证可复现，我们把以下信息写进转换脚本旁的记录文件：Toolkit 与运行库版本均为 1.6.0、asymmetric_quantized-8、quantized_algorithm=normal、optimization_level=3、校准集清单文件名与图片数量、ONNX 的哈希值。每次重新转换，必须重新跑上述四步验证，并把 mAP 与延迟记入同一份记录里。这样当"同样的图今天检测不出来"时，可以先排除"模型文件变了"。

> **面试要点**：被问"INT8 掉了多少精度，值不值"，答："mAP@0.5 从 0.86 降到 0.83，小目标病斑 Recall 降得最多，从 0.78 到 0.73。换来的是模型体积约一半、NPU 推理约快 1.6 倍。我的任务是秒到分钟级的灌溉建议，所以用多帧确认补偿召回，而不是退回 FP16。"

---

## 本章要点速记

1. $q=\operatorname{clip}(\operatorname{round}(x/s)+z)$，$\hat x=s(q-z)$；$s=(x_{\max}-x_{\min})/255$，$z=q_{\min}-x_{\min}/s$。
2. 未饱和时误差 $\le s/2$；饱和误差无此上界（20 → 12.7，误差 7.3）。
3. 非对称用满 256 档：ReLU $[0,6]$ 时分辨率是对称的 2 倍。
4. 逐通道 scale 让小幅值通道精度更高：同一权重 5.5% → 1.6% 误差。
5. 乘加：int32 累加，$M=s_xs_w/s_y$ 重量化；零点项展开。
6. 校准集决定激活范围：要有代表性、预处理一致；离群点可让步长变粗 10 倍。
7. 检测头输出范围差异大，共享 scale 会碾碎分数：拆分输出 + CPU 解码。
8. ONNX→RKNN：config → load_onnx → build(量化) → export；Toolkit 与运行库版本要匹配。
9. 转换≠训练；脚本不能证明 mAP/FPS，要自己测 FP32 vs INT8。

## 自测题

1. $s=0.05,\ z=0$，量化 $x=0.37$，求 $q$、$\hat x$、误差。
2. 数据范围 $[-1,\ 3]$，int8 非对称，求 $s,z$ 并验证两端。
3. 通道 A 范围 ±0.2，通道 B 范围 ±1.0，求逐通道 scale；权重 0.05 在 A 通道逐层 vs 逐通道量化各得多少？
4. 校准集里混入一张极亮图，使某层激活最大值从 5 变成 50，min–max 下步长变为原来的多少倍？
5. 为什么说"我写了 onnx2rknn.py"不能回答"你的 mAP 是多少"？

**参考答案**：
1. $0.37/0.05=7.4\to7$，$\hat x=0.35$，误差 0.02 $\le0.025$。
2. $s=4/255=0.015686$；$z=-128-\operatorname{round}(-1/0.015686)=-128+64=-64$（$-63.75\to-64$）；$x=-1\to\operatorname{round}(-63.75)-64=-128$ ✓；$x=3\to\operatorname{round}(191.25)-64=127$ ✓。
3. $s_A=0.2/127=0.001575$，$s_B=1/127=0.007874$；逐层用 $s_B$：$0.05/0.007874=6.35\to6\to0.04724$（误差 0.00276）；逐通道用 $s_A$：$0.05/0.001575=31.75\to32\to0.0504$（误差 0.0004）。
4. 10 倍（50/5）。
5. 转换不涉及评估；mAP 需要带标注验证集的评测记录。

## 相关链接

- 上一章：[06 YOLOv5 输出解码、NMS 与结构化事件](./06_YOLOv5输出解码_NMS与结构化事件.md)
- 下一章：[08 TCP 通信与协议设计](./08_TCP通信与协议设计.md)
- 基础：[模型量化与边缘部署 RKNN](../../02_基础知识/10_机器学习与深度学习/04_模型量化与边缘部署_RKNN.md)
- 八股：[模型部署与量化八股](../../03_八股知识/02_人工智能/06_模型部署与量化八股.md)
- 返回：[P4 README](./README.md)
