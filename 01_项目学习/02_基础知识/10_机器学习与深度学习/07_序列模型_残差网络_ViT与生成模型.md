# 序列模型、残差网络、ViT 与生成模型：深度学习的主流骨干

> **一句话定位**：[神经网络与反向传播](./02_神经网络与反向传播.md) 讲的是"一个网络怎么训练"，[卷积网络与目标检测](./03_卷积网络与目标检测_YOLO.md) 讲了 CNN。本章补齐现代深度学习的其余骨架：循环网络（RNN/LSTM/GRU）为什么会梯度消失、残差连接（ResNet）为什么能把网络做深、视觉 Transformer（ViT）怎么处理图像、对比学习怎么在没有标签时学表示，以及 VAE、GAN、扩散模型（diffusion）、流匹配（flow matching）这些生成模型——后两者正是后面"扩散策略""VLA"章节的底层。

**本章学完你能：**

- 写出 RNN 的前向和 BPTT 梯度，解释梯度消失/爆炸（手算 $0.9^{50}$），说明 LSTM/GRU 的门控怎样缓解；
- 手算一步 LSTM 单元；
- 解释残差连接 $y=x+F(x)$ 的梯度通路，手算 bottleneck 残差块相对朴素堆叠的参数量；
- 推导 ViT 的 token 数、参数量和注意力的 $O(N^2)$ 代价，说明与 CNN 的归纳偏置差异；
- 手算 InfoNCE 对比损失，解释 CLIP 的思想；
- 推导 VAE 的 ELBO，计算高斯 KL；
- 写出 DDPM 的前向加噪、训练目标、反向一步，手算数值，并区分 DDPM、DDIM 与流匹配；
- 用英文回答 "Why do residual connections help / LSTM vs Transformer / how does a diffusion model work / VAE vs GAN"。

**前置知识**：[神经网络与反向传播](./02_神经网络与反向传播.md)、[卷积网络与目标检测：YOLO](./03_卷积网络与目标检测_YOLO.md)、[Transformer、LLM 与 Agent](./05_Transformer_LLM与Agent_LangGraph.md)（注意力）、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)。

## 目录

1. 为什么需要这些骨架
2. 循环神经网络与梯度消失
3. LSTM 与 GRU（含手算）
4. 残差网络
5. 卷积的几个工程设计
6. 视觉 Transformer（ViT）
7. 自监督与对比学习
8. 自编码器与 VAE
9. GAN
10. 扩散模型（DDPM）
11. 流匹配与采样加速
12. 在本项目里怎么用
13. 常见错误与故障现象
14. 面试怎么问、怎么答
（补充）补充专题：归一化层、轻量卷积与序列训练细节
15. 本章要点速记
16. 自测题
17. 相关链接

---

## 1. 为什么需要这些骨架

- **时序数据**（IMU、关节轨迹、语言）：输入长度不定、前后相关。MLP 无法处理变长；RNN 用同一组权重按时间展开。
- **深度**：理论上网络越深表达力越强，但直接堆叠会出现"退化"（训练误差反而升高）。残差连接解决了它。
- **图像用 Transformer**：CNN 有很强的局部性/平移等变归纳偏置；ViT 把图像切成 patch 当作"词"，用注意力建模全局关系，数据足够时表现优秀。
- **没有标签的数据海量**：对比学习/掩码重建让模型自己学表示。
- **生成**：产生图像、轨迹、动作序列，需要对复杂分布建模——VAE、GAN、扩散各有取舍。

## 2. 循环神经网络与梯度消失

### 2.1 前向

$$h_t=\tanh(W_hh_{t-1}+W_xx_t+b),\qquad y_t=W_yh_t+b_y.$$
$h_t$ 是**隐状态**（记忆），同一组 $W$ 在所有时刻共享。

### 2.2 BPTT（沿时间的反向传播）

损失 $L=\sum_tL_t$。对早期隐状态求梯度：
$$\frac{\partial L_T}{\partial h_k}=\frac{\partial L_T}{\partial h_T}\prod_{t=k+1}^{T}\frac{\partial h_t}{\partial h_{t-1}},\qquad\frac{\partial h_t}{\partial h_{t-1}}=\operatorname{diag}(1-h_t^2)\,W_h.$$
连乘 $T-k$ 个雅可比矩阵。若它们的范数平均 $<1$，梯度指数衰减（**消失**）；$>1$，指数增长（**爆炸**）。

**手算**：每步范数 0.9，跨 50 个时间步：$0.9^{50}=0.0052$；每步 1.1：$1.1^{50}=117.4$（Python 复核）。意味着 RNN 很难学到相隔几十步的依赖。

### 2.3 应对

- **梯度爆炸**：梯度裁剪（clip 全局范数到阈值）；
- **梯度消失**：换结构（LSTM/GRU）、残差/跳连、更好的初始化（正交初始化 $W_h$）、ReLU 激活；
- **截断 BPTT**：每次只反传 $K$ 步，降低内存与不稳定。

## 3. LSTM 与 GRU（含手算）

### 3.1 LSTM（Long Short-Term Memory）

引入**细胞状态** $c_t$（"传送带"）和三个门，均由 sigmoid 输出 $[0,1]$：

$$
\begin{aligned}
f_t&=\sigma(W_f[h_{t-1},x_t]+b_f)\quad\text{遗忘门}\\
i_t&=\sigma(W_i[h_{t-1},x_t]+b_i)\quad\text{输入门}\\
o_t&=\sigma(W_o[h_{t-1},x_t]+b_o)\quad\text{输出门}\\
g_t&=\tanh(W_g[h_{t-1},x_t]+b_g)\quad\text{候选记忆}\\
c_t&=f_t\odot c_{t-1}+i_t\odot g_t\\
h_t&=o_t\odot\tanh(c_t)
\end{aligned}
$$

关键在 $c_t=f_t\odot c_{t-1}+\dots$：这是**加法更新**。$\partial c_t/\partial c_{t-1}=f_t$，只要遗忘门接近 1，梯度就几乎无衰减地沿时间流动，所以 LSTM 能记住长依赖。

### 3.2 手算一步（标量，示例）

设 $c_{t-1}=0.2$，各门的预激活（已算好）：$z_f=0.5$，$z_i=1.0$，$z_o=-0.5$，$z_g=0.8$。

- $f=\sigma(0.5)=0.6225$，$i=\sigma(1.0)=0.7311$，$o=\sigma(-0.5)=0.3775$，$g=\tanh(0.8)=0.6640$；
- $c_t=0.6225\times0.2+0.7311\times0.6640=0.1245+0.4855=0.6099$；
- $h_t=0.3775\times\tanh(0.6099)=0.3775\times0.5441=0.2054$（Python 复核 $c_t=0.6099$，$h_t=0.2054$）。

读法：遗忘门保留了旧记忆 62% 的成分，输入门写入了新信息 73% 的成分。

### 3.3 GRU

只有更新门 $z_t$ 和重置门 $r_t$，没有独立细胞状态：$h_t=(1-z_t)\odot h_{t-1}+z_t\odot\tilde h_t$。参数更少（约为 LSTM 的 3/4），效果常相近。

**参数量**：LSTM 参数 $=4\,(d_h(d_h+d_x)+d_h)$。$d_h=256$，$d_x=128$：$4\times(256\times384+256)=394{,}240$；GRU 为 $3\times\ldots=295{,}680$（Python 复核 LSTM 394240）。

### 3.4 为什么现在 Transformer 更常见

RNN 必须按时间顺序计算（难并行），长依赖仍有限；Transformer 用注意力任意两位置直接连接、并行训练。但 RNN/LSTM 在**低延迟、小模型、流式推理**场景仍有价值（嵌入式上的状态估计、控制中的记忆）。

## 4. 残差网络

### 4.1 退化问题

实验观察：把网络从 20 层加到 56 层，**训练误差**也更高——不是过拟合，而是深层网络难以优化。

### 4.2 残差块

不再学习 $H(x)$，而学习残差 $F(x)=H(x)-x$：
$$y=x+F(x).$$
**梯度**：$\dfrac{\partial L}{\partial x}=\dfrac{\partial L}{\partial y}\Big(I+\dfrac{\partial F}{\partial x}\Big)$。那个恒等项 $I$ 保证梯度至少有一条"高速公路"直达浅层，不被连乘衰减。展开 $L$ 层后 $x_L=x_l+\sum_iF_i$，梯度含 $\partial L/\partial x_L$ 的直接项。

直觉：**恒等映射是一个好的起点**，要学的只是对它的小修正；如果某层没用，让 $F\to0$ 即可。

### 4.3 Bottleneck（瓶颈）块与参数量

$3\times3$ 卷积、通道 $C$：参数 $9C^2$。

**手算**：通道 256 的两层 $3\times3$ 卷积：$2\times9\times256^2=1{,}179{,}648$。瓶颈结构（$1\times1$ 降到 64 → $3\times3$ → $1\times1$ 升回 256）：$256\times64+9\times64^2+64\times256=16{,}384+36{,}864+16{,}384=69{,}632$，约少 17 倍（Python 复核）。这让网络可以做到上百层仍保持可训练的计算量。

### 4.4 与归一化的搭配

残差 + BatchNorm/LayerNorm 是几乎所有现代网络的标准配置。Transformer 的每个子层同样是 $x+\text{Sublayer}(\text{LN}(x))$（Pre-LN）。**YOLO 的骨干（CSP、C2f 等）**里也大量用残差/跳连（具体结构因版本而异，未核对）。

## 5. 卷积的几个工程设计

- **感受野**：堆叠 $n$ 层 $3\times3$（步长 1），感受野 $2n+1$；3 层是 7（Python 复核），与一个 $7\times7$ 卷积一样大，但参数 $3\times9C^2=27C^2<49C^2$，且多了非线性；
- **步长/池化**：下采样，扩大感受野、降低计算；
- **空洞卷积（dilated）**：不增加参数扩大感受野；
- **深度可分离卷积（depthwise separable）**：把标准卷积拆成逐通道的空间卷积 + $1\times1$ 通道混合，参数/计算降到约 $1/C_{out}+1/9$，MobileNet 一类轻量网络的核心，适合边缘设备（见 [模型量化与边缘部署：RKNN](./04_模型量化与边缘部署_RKNN.md)）；
- **特征金字塔（FPN）**：融合多尺度特征，利于检测小目标。

## 6. 视觉 Transformer（ViT）

### 6.1 流程

1. 把图像切成 $P\times P$ 的 patch，展平并线性投影成 $d$ 维向量（这就是 token）；
2. 加上**位置编码**（因为注意力本身对顺序无感）；
3. 在序列最前面加一个可学习的 `[CLS]` token；
4. 送入 $L$ 层 Transformer encoder（多头自注意力 + MLP + 残差 + LN）；
5. 用 `[CLS]` 的输出做分类（或用所有 token 做密集预测）。

### 6.2 手算

输入 $224\times224\times3$，patch $16\times16$：token 数 $(224/16)^2=196$，加 `[CLS]` 共 $N=197$。patch 投影到 $d=768$ 的参数：$16\times16\times3\times768=589{,}824$（Python 复核）。注意力矩阵 $N^2=197^2=38{,}809$ 个元素/头。若把分辨率翻倍成 $448$，token 变成 $784$，注意力成本变成原来的约 $16$ 倍——所以**高分辨率下 ViT 昂贵**，需要窗口注意力（Swin）或降采样。

### 6.3 与 CNN 对比

| | CNN | ViT |
|---|---|---|
| 归纳偏置 | 局部性、平移等变 | 弱（需要大量数据学出来） |
| 小数据 | 强 | 弱（常需预训练） |
| 全局建模 | 靠深度堆叠 | 第一层就是全局 |
| 计算 | 随像素线性 | 随 token 数二次 |
| 部署 | 成熟（量化工具链好） | 较新，注意力算子支持不一（未核对） |

现代视觉-语言和 VLA 模型常用 ViT 作为视觉编码器（见 [大语言模型预训练、微调、RAG 与 VLA](./08_大语言模型预训练_微调_RAG与VLA.md)）。

## 7. 自监督与对比学习

### 7.1 想法

没有标签时，构造"伪任务"：同一张图的两个增强视图应该靠近，不同图应该远离。

### 7.2 InfoNCE 损失

对样本 $i$ 的正样本相似度 $s_{ii}$，与批内负样本 $s_{ij}$，温度 $\tau$：
$$L_i=-\log\frac{\exp(s_{ii}/\tau)}{\sum_j\exp(s_{ij}/\tau)}.$$
相当于 $N$ 类分类（找出哪个是正样本）。

**手算**：相似度 $[0.9,0.1,0.2]$，$\tau=0.1$：logits $[9,1,2]$，$L=-\log\frac{e^9}{e^9+e^1+e^2}=0.00125$（几乎完美）。若模型没学好，相似度 $[0.5,0.4,0.45]$：logits $[5,4,4.5]$，$L=0.680$（Python 复核）。温度越小，对难负样本越敏感，分布越尖。

### 7.3 CLIP

图像编码器与文本编码器，对 $N$ 对图文做双向 InfoNCE：同一对的相似度大，其余小。训练后可零样本分类（用类别名生成文本，比较相似度），并为 VLA 提供语义对齐的视觉特征（细节见后续章节）。

### 7.4 掩码重建（MAE / BERT 式）

随机遮住大部分 patch/词，让模型预测被遮部分，学到上下文表示。

## 8. 自编码器与 VAE

### 8.1 自编码器

编码器 $z=E(x)$ 压缩，解码器 $\hat x=D(z)$ 重建，最小化 $\|x-\hat x\|^2$。能降维、去噪，但 $z$ 的空间不连续，**不能直接采样生成**。

### 8.2 VAE

让编码器输出分布 $q_\phi(z|x)=\mathcal N(\mu,\sigma^2)$，并让它接近先验 $p(z)=\mathcal N(0,I)$。最大化**证据下界（ELBO）**：
$$\log p(x)\ge\underbrace{\mathbb E_{q(z|x)}[\log p_\theta(x|z)]}_{\text{重建}}-\underbrace{\mathrm{KL}\big(q(z|x)\,\|\,p(z)\big)}_{\text{正则}}.$$
**重参数化技巧**：$z=\mu+\sigma\odot\epsilon$，$\epsilon\sim\mathcal N(0,I)$，使采样可导。

一维高斯的 KL 闭式：
$$\mathrm{KL}\big(\mathcal N(\mu,\sigma^2)\|\mathcal N(0,1)\big)=\tfrac12\big(\mu^2+\sigma^2-1-\ln\sigma^2\big).$$
**手算**：$\mu=1,\sigma=0.5$：$\tfrac12(1+0.25-1-\ln0.25)=\tfrac12(0.25+1.386)=0.818$（Python 复核）。$\mu=0,\sigma=1$ 时 KL=0。

特点：训练稳定、有隐空间可插值，但生成偏模糊（均方重建）。

## 9. GAN

生成器 $G(z)$ 造假，判别器 $D(x)$ 辨真假，极小极大博弈：
$$\min_G\max_D\ \mathbb E_{x}[\log D(x)]+\mathbb E_z[\log(1-D(G(z)))].$$
优点：样本清晰。缺点：训练不稳定、模式坍塌（mode collapse）、没有似然。常见改进：WGAN、谱归一化等。在机器人里，GAN 曾用于域适应和 sim-to-real 图像转换；近年扩散模型逐渐取代它作为主流生成模型。

## 10. 扩散模型（DDPM）

### 10.1 想法

**前向过程**：逐步给数据加高斯噪声，直到变成纯噪声；**反向过程**：学一个网络，把噪声一步步去掉。生成 = 从噪声出发反向去噪。

### 10.2 前向（有闭式解）

$$q(x_t|x_{t-1})=\mathcal N\big(\sqrt{1-\beta_t}\,x_{t-1},\ \beta_tI\big).$$
令 $\alpha_t=1-\beta_t$，$\bar\alpha_t=\prod_{s\le t}\alpha_s$，可以**一步直接采样任意 $t$**：
$$x_t=\sqrt{\bar\alpha_t}\,x_0+\sqrt{1-\bar\alpha_t}\,\epsilon,\quad\epsilon\sim\mathcal N(0,I).$$
线性 $\beta$ 调度（$10^{-4}\to0.02$，$T=1000$）：$\bar\alpha_1=0.9999$，$\bar\alpha_{100}=0.897$，$\bar\alpha_{500}=0.0786$，$\bar\alpha_{1000}=4\times10^{-5}$（Python 复核）——末端几乎是纯噪声。

### 10.3 训练

网络 $\epsilon_\theta(x_t,t)$ 预测所加的噪声，损失极简单：
$$L=\mathbb E_{x_0,t,\epsilon}\big\|\epsilon-\epsilon_\theta(x_t,t)\big\|^2.$$
**手算**：设 $\bar\alpha_t=0.64$（任意取值的示例），$x_0=1.0$，$\epsilon=0.5$：$x_t=0.8\times1+0.6\times0.5=1.1$。若网络预测正确 $\hat\epsilon=0.5$，则可反推 $\hat x_0=(x_t-\sqrt{1-\bar\alpha_t}\hat\epsilon)/\sqrt{\bar\alpha_t}=(1.1-0.3)/0.8=1.0$ ✓。预测错误就得到错误的 $\hat x_0$，损失即为惩罚。

### 10.4 反向一步（采样）

$$x_{t-1}=\frac1{\sqrt{\alpha_t}}\Big(x_t-\frac{\beta_t}{\sqrt{1-\bar\alpha_t}}\epsilon_\theta(x_t,t)\Big)+\sigma_tz,\quad z\sim\mathcal N(0,I).$$
**手算**：线性调度下 $t=500$：$\alpha_t=0.98996$，$\beta_t=0.01004$，$\bar\alpha_t=0.0786$，$\sqrt{1-\bar\alpha_t}=0.9599$。若 $x_t=1.1$，$\epsilon_\theta=0.5$：均值 $=\dfrac{1}{0.99496}(1.1-\dfrac{0.01004}{0.9599}\times0.5)=1.1003$（Python 复核；这一步变化很小，所以需要走上千步）。

### 10.5 条件生成

把条件 $c$（语言、图像、机器人状态）作为网络输入：$\epsilon_\theta(x_t,t,c)$。**分类器无引导（classifier-free guidance）**：训练时随机丢弃条件，采样时 $\hat\epsilon=\epsilon_\theta(x_t,\varnothing)+w(\epsilon_\theta(x_t,c)-\epsilon_\theta(x_t,\varnothing))$，$w>1$ 增强条件影响。

### 10.6 优缺点

- 优点：训练稳定、覆盖多模态分布（不会塌缩到一个模式）、质量高；
- 缺点：采样慢（几十到上千次网络前向），实时机器人控制需要加速。

## 11. 流匹配与采样加速

- **DDIM**：用确定性采样跳步，几十步即可；
- **蒸馏 / consistency model**：把多步压缩成 1–几步（原理概述，具体算法未核对）；
- **流匹配（flow matching）**：学习一个速度场 $v_\theta(x_t,t)$，把简单分布沿直线路径搬运到数据分布。训练样本：$x_t=(1-t)x_0+tx_1$，目标速度 $x_1-x_0$。**手算**：$x_0=0$（噪声），$x_1=2$（数据），$t=0.25$：$x_t=0.5$，目标速度 $2.0$。采样时用 ODE 积分，通常 10 步左右即可，比 DDPM 快。部分 VLA 模型的动作头采用流匹配（见 [大语言模型预训练、微调、RAG 与 VLA](./08_大语言模型预训练_微调_RAG与VLA.md)，细节未核对）。

## 12. 在本项目里怎么用

- **P4 智能园艺**：YOLO 骨干是残差/跳连 + 多尺度融合的 CNN；部署到 RK3588 时的量化与算子支持问题，与第 5、6 节的结构选择相关（见 [卷积网络与目标检测：YOLO](./03_卷积网络与目标检测_YOLO.md)、[模型量化与边缘部署：RKNN](./04_模型量化与边缘部署_RKNN.md)）。CV 写的是 YOLO 检测与部署，不要说成自己训练了 ViT 或扩散模型。
- **P2 遥操作 → 数据采集**：leader–follower 示教和 VR 遥操作产生的 $(观测, 动作)$ 轨迹，正是**模仿学习**（行为克隆、扩散策略）的训练数据；本章扩散模型是 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md) 的基础。这是"如果继续读研你想做什么"的自然延伸，但 CV 没有写训练策略，不可当作已做过的工作。
- **P6 / P1 的时序数据**：IMU/关节时序若需要学习式滤波，可用 LSTM/GRU 或轻量 Transformer；工程上优先用经典滤波（互补滤波、卡尔曼）。

## 13. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| RNN loss 变成 NaN | 梯度爆炸 | 梯度裁剪，降低学习率 |
| 长序列学不到远处依赖 | 梯度消失 | 换 LSTM/GRU/Transformer，截断 BPTT 并加记忆 |
| 深网络训练误差比浅网高 | 退化 | 加残差连接与归一化 |
| ViT 小数据过拟合/不收敛 | 弱归纳偏置 | 预训练权重、数据增强、更小模型 |
| VAE 生成图模糊 | 均方重建与 KL 的折中 | 调 KL 权重 $\beta$，改用扩散 |
| GAN 模式坍塌 | 判别器过强/训练不平衡 | WGAN、谱归一化 |
| 扩散采样太慢 | 步数多 | DDIM、流匹配、蒸馏 |
| 对比学习塌缩（所有输出一样） | 缺少负样本/正则 | 足够负样本，或 BYOL/SimSiam 类设计 |

## 14. 面试怎么问、怎么答

**Q1. Why do residual connections make deep networks easier to train?**

> A residual block computes y = x + F(x), so the gradient has an identity path: the derivative of y with respect to x is I plus the derivative of F. Gradients reach early layers without being multiplied through many weight matrices, and a block that is not useful can simply learn F close to zero. This lets us train very deep networks, and together with normalisation it is now standard in CNNs and Transformers.

中文：$y=x+F(x)$，梯度有恒等通路 $I+\partial F/\partial x$，不会被一串权重连乘衰减；无用的块让 $F\to0$ 即可；配合归一化成为标准配置。

**Q2. Why do vanilla RNNs struggle with long-range dependencies, and how does LSTM help?**

> Backpropagation through time multiplies one Jacobian per step, so the gradient shrinks or explodes exponentially. An LSTM adds a cell state updated additively, c_t = f_t * c_{t-1} + i_t * g_t, so with the forget gate near one the gradient flows almost unchanged. Gradient clipping handles the exploding case.

中文：BPTT 连乘雅可比，梯度指数消失/爆炸；LSTM 的细胞状态加法更新，遗忘门接近 1 时梯度几乎无损；爆炸用梯度裁剪。

**Q3. Explain how a diffusion model generates data.**

> Training: I add Gaussian noise to a data sample at a random level t using a closed-form formula and train a network to predict that noise. Sampling: I start from pure noise and repeatedly remove the predicted noise, step by step, until I reach a clean sample. Conditioning, for example on an observation, is done by feeding it to the network. It covers multi-modal distributions well but needs many network evaluations, which is why people use DDIM or flow matching to speed it up.

中文：训练：随机噪声强度下用闭式加噪，让网络预测所加噪声；采样：从纯噪声开始逐步去噪；条件（如观测）输入网络；擅长多模态分布但步数多，常用 DDIM/流匹配加速。

**Q4. VAE versus GAN versus diffusion?**

> A VAE is stable and gives a smooth latent space, but samples tend to be blurry. A GAN gives sharp samples but training is unstable and can collapse modes. Diffusion models are stable and cover the data distribution well with high quality, at the cost of slow sampling. For robot policies I would favour diffusion-style heads because they represent multi-modal action distributions.

中文：VAE 稳定、隐空间平滑但偏模糊；GAN 清晰但不稳定易模式坍塌；扩散稳定、覆盖好、质量高但采样慢；机器人策略因多模态动作分布更适合扩散式动作头。

**Q5. What is the inductive bias difference between CNN and ViT?**

> A CNN bakes in locality and translation equivariance, so it learns efficiently from limited data. A ViT has almost no such bias and relies on data and pre-training, but it models global relations from the first layer and scales well with data. The attention cost grows quadratically with the number of patches.

中文：CNN 内置局部性与平移等变，小数据高效；ViT 偏置弱，依赖大数据/预训练，但第一层即全局建模，扩展性好；注意力成本随 patch 数二次增长。

## 补充专题：归一化层、轻量卷积与序列训练细节

### 三种归一化的区别（含 LayerNorm 手算）

归一化的共同形式：$\hat x=\dfrac{x-\mu}{\sqrt{\sigma^2+\epsilon}}$，再乘可学习的缩放 $\gamma$ 加平移 $\beta$。区别只在于"在哪些维度上统计 $\mu,\sigma$"：

| 方法 | 统计范围 | 依赖批大小 | 典型用途 |
|---|---|---|---|
| BatchNorm | 同一通道，跨批次与空间 | 是（小批不稳；训练与推理行为不同） | CNN |
| LayerNorm | 同一个样本，跨所有特征 | 否 | Transformer、RNN |
| GroupNorm | 同一样本，通道分组 | 否 | 小批量检测/分割、扩散策略中的视觉骨干 |
| RMSNorm | 只除以均方根，不减均值 | 否 | 部分大语言模型 |

**手算 LayerNorm**：某个 token 的特征向量 $x=[1,2,3,4]$。$\mu=2.5$，$\sigma^2=1.25$，$\epsilon=10^{-5}$：$\hat x=\dfrac{x-2.5}{\sqrt{1.25001}}=[-1.342,-0.447,0.447,1.342]$（Python 复核），均值 0，方差约 1。无论输入尺度如何，归一化后尺度一致，因此残差网络里每层的输入量级稳定，梯度更健康。BatchNorm 在推理时用训练期间累计的滑动均值方差，因此**部署前要设为 eval 模式**，且在量化/融合时常把 BN 折叠进前一个卷积（见 [模型量化与边缘部署：RKNN](./04_模型量化与边缘部署_RKNN.md)）；批量很小的场景（机器人在线学习、模仿学习小批量）BN 的统计量不可靠，常改用 GroupNorm。

### 深度可分离卷积的参数对比

标准 $3\times3$ 卷积，输入输出通道均为 128：$9\times128\times128=147{,}456$ 个参数。深度可分离：先逐通道 $3\times3$（$9\times128=1{,}152$），再 $1\times1$ 通道混合（$128\times128=16{,}384$），共 $17{,}536$，约为标准的 $1/8.4$（Python 复核）。计算量比例相同。代价是表达能力下降一些，需要更宽或更多层补偿。移动端和 RK3588 这类 NPU 上，是否真的更快取决于 NPU 对深度卷积的支持效率，实测为准（未核对）。

### 序列训练的几个细节

- **Teacher forcing**：训练时每一步的输入用真实上一步，而不是模型自己的预测，训练稳定、可并行；但推理时用自己的预测，产生**曝光偏差**（exposure bias）——一个错误会影响后续。这与模仿学习里的分布偏移同源（见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)）；
- **填充与掩码**：批内序列不等长，需要 padding 并在注意力/损失中用 mask 忽略填充位置；
- **序列长度与显存**：RNN 的显存随长度线性增长（需存每步激活以做 BPTT），Transformer 随长度二次增长（注意力矩阵）；
- **双向 vs 因果**：离线任务可用双向（看未来），实时控制必须因果；
- **状态重置**：在线运行的 RNN/LSTM 策略，每个新回合要重置隐状态，训练时的截断 BPTT 窗口和部署时的隐状态延续方式要一致，否则部署行为偏离训练。

### 迁移学习与微调骨干

在小数据任务（如 P4 的作物病害检测）里，常用在大数据集上预训练的骨干，换掉最后的分类/检测头，分两阶段训练：先**冻结骨干**只训新头，几个 epoch 后**解冻**以较小学习率（如头的 1/10）微调。数据增强（翻转、色彩抖动、随机裁剪、Mosaic 等，视检测器而定）能显著缓解过拟合。经验判断：训练集与验证集的 loss 差距变大时，优先加增强和正则，而不是盲目加深网络。

### 自监督学习的实际价值

机器人领域未标注数据很容易获得（每天都有视频与传感器日志），而标注昂贵。对比学习或掩码重建预训练的视觉编码器，常用于下游的抓取、导航、VLA 视觉骨干；其好坏要靠下游任务的成功率评估，不能只看预训练 loss。

### 补充：如何在面试里比较这些骨架

被问到"CNN、RNN、Transformer 各适合什么"时，可以按三个轴回答：**数据结构**（网格→CNN，序列→RNN/Transformer，图→GNN）、**依赖距离**（局部→CNN，远距离→Transformer）、**延迟与资源**（流式、小模型→RNN/轻量 CNN，大规模并行→Transformer）。再补一句工程现实："在边缘设备上，我首先看算子是否被 NPU 支持、量化后精度是否可接受，而不是只看论文指标"。这回应了 P4 部署 YOLO 的经历。对生成模型同理：看**样本质量、多模态覆盖、采样速度、训练稳定性**四项，机器人策略往往最看重多模态覆盖与采样速度的折中。

### 补充：用自己的话讲清"为什么深度学习有效"

可以用三点回答：(1) **表示学习**：网络逐层把原始输入变成对任务有用的特征，无需手工设计；(2) **规模与数据**：参数多、数据多，加上 GPU 并行与反向传播，使得拟合复杂函数成为可能；(3) **归纳偏置**：卷积、注意力、残差等结构在函数空间里编码了对世界的先验（局部性、全局关联、恒等通路），让学习更高效。同时要承认局限：需要大量数据与算力、可解释性弱、对分布外输入脆弱、没有安全保证——这就是为什么机器人系统中学习模块通常放在"感知与高层决策"，而把稳定性关键的底层交给经典控制。这样的回答体现了你既懂也克制。

### 补充：选模型时的三个提问

面对一个新任务，先问：数据长什么样（图像、序列、点云、图）？数据有多少（几百、几万、百万以上）？推理预算是多少（毫秒级边缘设备，还是秒级云端）？数据少且是图像，优先预训练的 CNN 骨干加微调；数据多且需要全局建模，考虑 Transformer；要在微控制器上跑，考虑小型 CNN 或经典方法。把"选结构"变成回答这三个问题，比背网络名称更有说服力。再加一条工程判断：模型是否有成熟的量化与部署工具链，会直接影响项目周期，这也是很多机器人项目选择 YOLO 一类成熟检测器而不是追逐最新结构的原因。

## 15. 本章要点速记

1. RNN：$h_t=\tanh(W_hh_{t-1}+W_xx_t+b)$；BPTT 连乘雅可比 → 消失/爆炸；裁剪治爆炸。
2. LSTM：$c_t=f\odot c_{t-1}+i\odot g$ 加法更新，遗忘门近 1 保梯度；GRU 参数约 3/4。
3. 残差：$y=x+F(x)$，梯度 $I+\partial F/\partial x$；bottleneck 省 ~17 倍参数。
4. 3 层 $3\times3$ 感受野 7，参数少于 $7\times7$。
5. ViT：patch → token + 位置编码 + `[CLS]`；$224/16\Rightarrow196$ tokens；注意力 $O(N^2)$。
6. InfoNCE = 在批内找出正样本的分类；温度小则更尖。
7. VAE：ELBO = 重建 − KL；重参数化 $z=\mu+\sigma\epsilon$。
8. DDPM：$x_t=\sqrt{\bar\alpha_t}x_0+\sqrt{1-\bar\alpha_t}\epsilon$；损失 $\|\epsilon-\epsilon_\theta\|^2$。
9. 流匹配：$x_t=(1-t)x_0+tx_1$，目标速度 $x_1-x_0$，ODE 采样更快。
10. 机器人策略偏爱扩散/流匹配头，因为动作分布多模态。

## 16. 自测题

1. 每步梯度范数 0.95，跨 100 步剩多少？（$0.95^{100}=0.0059$。）
2. LSTM 中遗忘门恒为 0 会怎样？（$c_t=i_t\odot g_t$，丢失所有历史，退化成无记忆。）
3. 通道 128 的两个 $3\times3$ 卷积参数量？（$2\times9\times128^2=294{,}912$。）
4. $\mu=2,\sigma=1$ 的 KL（对 $\mathcal N(0,1)$）？（$\tfrac12(4+1-1-0)=2$。）
5. 为什么 DDPM 训练可以随机取任意 $t$ 而不用走完整条链？（前向过程有闭式解 $x_t=\sqrt{\bar\alpha_t}x_0+\dots$。）
6. ViT 输入 $384\times384$，patch 16，token 数（不含 CLS）？（$24^2=576$。）

## 17. 相关链接

- 上一章：[强化学习与具身智能入门](./06_强化学习与具身智能入门.md)
- 下一章：[大语言模型预训练、微调、RAG 与 VLA](./08_大语言模型预训练_微调_RAG与VLA.md)
- 相关：[神经网络与反向传播](./02_神经网络与反向传播.md)、[卷积网络与目标检测：YOLO](./03_卷积网络与目标检测_YOLO.md)、[模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)
- 模块说明：[10_机器学习与深度学习 README](./README.md)
