# 基于模型的强化学习、离线 RL 与 MPC 结合

> **一句话定位**：[强化学习进阶](./09_强化学习进阶_策略梯度_PPO_SAC.md) 讲的是"模型无关"（model-free）的 PPO/SAC：不学环境，靠海量交互。本章讲另外两条减少真机交互的路线——**基于模型的强化学习**（model-based RL，学一个动力学模型，再用它规划或生成数据）和**离线强化学习**（offline RL，只用已有数据集学策略，不再与环境交互）——以及它们与你熟悉的 MPC 如何结合（学习动力学、学习价值函数做终端代价）。

**本章学完你能：**

- 说清基于模型与模型无关 RL 的取舍，解释"模型误差累积"并手算其放大；
- 手算 Dyna-Q 的一次"真实经验 + 规划回放"，说明 Dyna、MBPO、PETS、Dreamer 各自的核心想法；
- 解释概率集成（ensemble）如何区分认知不确定性与偶然不确定性；
- 手算 CEM 与 MPPI 的一次采样更新，理解"随机采样 MPC"；
- 手算带终端价值的 MPC，说明为什么"短视野 + 学到的价值"可以逼近长视野；
- 解释离线 RL 的分布偏移与外推误差，手算 CQL 的保守惩罚与 IQL 的期望分位（expectile）损失；
- 对比 RL 与 MPC 并给出组合方案；
- 用英文回答 "Model-based vs model-free / why does offline RL fail / what is CQL / how to combine learning with MPC" 及追问；
- 诚实说明本章与七个项目的关系：项目里的 MPC（P5）和 LQR（P1/P7）使用的是解析模型，学习部分均是延伸。

**前置知识**：[强化学习进阶](./09_强化学习进阶_策略梯度_PPO_SAC.md)、[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)、[神经网络与反向传播](./02_神经网络与反向传播.md)、[概率图模型、贝叶斯方法与高斯过程](./12_概率图模型_贝叶斯方法_高斯过程.md)（不确定性）、[模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)（分布偏移）。

## 目录

1. 为什么要学这一章
2. 基于模型的 RL：基本框架
3. 模型误差累积（手算）
4. Dyna：用模型生成"想象经验"（手算）
5. 概率集成、MBPO、PETS 与 Dreamer 的思想
6. 与 MPC 结合：随机采样 MPC、终端价值
7. 离线 RL：分布偏移与外推误差
8. 离线 RL 的三类方法与手算（CQL、IQL）
9. 选择与组合：RL、MPC、模仿学习
10. 在七个项目里怎么用（含诚实边界）
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记
14. 自测题
15. 相关链接

---

## 1. 为什么要学这一章

真机上做强化学习有三个现实障碍：交互慢（一步一步地走）、危险（探索性动作会摔机器人）、昂贵（磨损、人力）。两条应对思路：

1. **别太频繁地和真环境打交道**：学一个环境模型，在"脑内"推演，这是基于模型的 RL；
2. **别再和真环境打交道**：用以前录好的数据（遥操作日志、历史运行记录）训练，这是离线 RL。

对于已经有 MPC 经验的你，这一章提供了一座桥：MPC 的核心是"模型 + 优化"，本章讲的是"模型不再手推，而是学出来；代价/价值也可以学出来"。

> **注意**：本章所列算法的具体细节、超参数与论文结果，除标注"手算"的数值外，均属概念性介绍；具体文献只写名称与思想，未逐篇核对出处与年份。

## 2. 基于模型的 RL：基本框架

### 2.1 问题与符号

| 符号 | 含义 |
|---|---|
| $s_t, a_t, r_t$ | 状态、动作、奖励 |
| $f_\theta(s,a)$ | 学到的动力学模型，预测 $s_{t+1}$（确定性）或其分布 |
| $\hat r_\theta(s,a)$ | 学到的奖励模型（有时已知则不用学） |
| $\mathcal D=\{(s,a,r,s')\}$ | 真实经验数据集 |
| $H$ | 规划 / 想象视野（步数） |
| $\gamma$ | 折扣因子 |

### 2.2 基本循环

1. 用某个策略与真环境交互，收集 $\mathcal D$；
2. 监督学习拟合模型：$\min_\theta\ \mathbb E_{(s,a,s')\sim\mathcal D}\big[\lVert f_\theta(s,a)-s'\rVert^2\big]$（或最大化概率模型的对数似然）；
3. 用模型"想象"：从模型里采样轨迹，用于规划（选动作）或训练策略/价值；
4. 回到步骤 1，持续迭代。

与模型无关 RL 的对比：

| 维度 | 模型无关（PPO/SAC） | 基于模型 |
|---|---|---|
| 真机样本需求 | 大 | 小（常见量级差一个到两个数量级，视任务而定，未核对） |
| 渐近性能 | 通常较高 | 受模型偏差限制，可能较低 |
| 计算 | 训练重但推理轻 | 规划时推理重（需要在线规划） |
| 主要风险 | 样本效率差 | 模型误差被策略"利用"（exploitation） |
| 可解释性 | 低 | 模型可检查、可复用于不同任务 |

> **面试要点**：基于模型方法的天敌是**模型偏差被优化器利用**：策略会找到模型里的"漏洞"（模型错误地认为某动作很好），在真机上失败。几乎所有改进（集成、短视野回放、保守性）都在对付这一点。

### 2.3 模型怎么学

- **确定性网络**：预测 $\Delta s=s'-s$（预测增量而不是下一状态绝对值，使学习目标尺度更小、更平稳）；
- **概率网络**：输出高斯 $\mathcal N(\mu_\theta,\Sigma_\theta)$，损失为负对数似然，能表达偶然不确定性（噪声）；
- **集成**：训练 $M$ 个不同初始化 / 数据子集的模型，成员之间的分歧反映认知不确定性（见 5.1）；
- **物理 + 残差**：已知物理模型 $f_{\text{phys}}$，只学残差 $f_\theta=f_{\text{phys}}+g_\theta$。这在机器人里最实用，数据需求最少；
- **潜空间模型**：图像观测下，先编码到低维潜变量，再在潜空间学动力学（Dreamer 类，见 5.3）。

## 3. 模型误差累积（手算）

### 3.1 为什么多步预测会越来越差

单步模型只在"数据分布内"训练过。多步预测时，第 $k$ 步的输入是第 $k-1$ 步的预测，已经带了误差；这个误差又作为新的输入被模型放大，还可能把状态推到训练数据没覆盖的区域（分布偏移，与模仿学习里的复合误差同源）。设单步误差上界为 $\varepsilon$，动力学对状态的利普希茨常数为 $L$，则 $k$ 步后误差大致满足

$$e_k\le L\,e_{k-1}+\varepsilon\ \Rightarrow\ e_k\le\varepsilon\,\frac{L^k-1}{L-1}\quad(L\ne1).$$

$L>1$（不稳定、混沌系统）时按指数增长；$L=1$ 时线性增长 $k\varepsilon$；$L<1$（稳定系统）时饱和于 $\varepsilon/(1-L)$。这是经验性的上界推导，不是紧的界。

### 3.2 完整手算（示例，非项目实测）

标量系统 $x_{k+1}=ax_k$，初值 $x_0=1$。真实 $a=1.05$（略不稳定），学到的模型 $\hat a=1.07$（系数误差 0.02）。

| 步数 $k$ | 真实 $x_k=1.05^k$ | 模型 $\hat x_k=1.07^k$ | 绝对误差 | 相对误差 |
|---|---|---|---|---|
| 1 | 1.0500 | 1.0700 | 0.0200 | 1.9% |
| 5 | 1.2763 | 1.4026 | 0.1263 | 9.9% |
| 10 | 1.6289 | 1.9672 | 0.3383 | 20.8% |

一步相对误差不到 2%，十步后就到 21%：误差随步数几乎指数放大（Python 复核）。作为对照，稳定系统 $x_{k+1}=0.95x_k+0.5$（真实）与 $\hat a=0.97,\hat b=0.45$：第 1 步误差 0.03，第 10 步 0.065，误差饱和而不是爆炸。结论：**不稳定或高度非线性对象，模型视野必须短**。

### 3.3 应对策略

1. **短视野回放**：只从真实状态出发做 $k=1\sim5$ 步的想象，不做长回放；
2. **集成 + 不确定性惩罚**：成员分歧大的区域，对奖励扣分或截断轨迹；
3. **多步损失训练**：训练时直接最小化多步预测误差，而不只是单步；
4. **滚动时域**：只执行规划序列的第一步，然后重新规划（这就是 MPC 的反馈思想，能不断修正模型误差）；
5. **物理先验 + 残差**：缩小模型需要学的部分，降低 $\varepsilon$。

## 4. Dyna：用模型生成"想象经验"（手算）

Dyna 架构（概念性介绍）：每得到一条真实转移，既更新价值/策略，又把它存入模型，然后用模型**再随机回放若干步**做同样的更新。表格版 Dyna-Q 的更新为

$$Q(s,a)\leftarrow Q(s,a)+\alpha\big[r+\gamma\max_{a'}Q(s',a')-Q(s,a)\big].$$

**手算（示例，非项目实测）**：链式环境 $s_0\xrightarrow{a}s_1\xrightarrow{a}s_2$（终止状态），只有最后一步奖励 $r=1$，其余为 0；$\alpha=0.5$，$\gamma=0.9$，$Q$ 初值全 0，只有一个动作 $a$。

**真实经验 1**：$(s_0,a,0,s_1)$。目标 $0+0.9Q(s_1)=0$，$Q(s_0)=0$。

**真实经验 2**：$(s_1,a,1,s_2)$，终止。目标 $=1$，$Q(s_1)=0+0.5\times(1-0)=0.5$。

**规划（模型已存两条转移，随机回放三次，不需要再和环境交互）：**

1. 回放 $(s_0,a,0,s_1)$：目标 $0.9\times0.5=0.45$，$Q(s_0)=0+0.5\times0.45=0.225$；
2. 回放 $(s_1,a,1,s_2)$：目标 1，$Q(s_1)=0.5+0.5\times0.5=0.75$；
3. 再次回放 $(s_0,a,0,s_1)$：目标 $0.9\times0.75=0.675$，$Q(s_0)=0.225+0.5\times(0.675-0.225)=0.45$。

仅 2 条真实经验，通过 3 次想象回放，$Q(s_0)$ 从 0 变成 0.45，**价值沿链条反向传播得更快**。若没有规划，$Q(s_0)$ 要等下一次真实走到 $s_0$ 才更新一次。Python 复核：$Q(s_0)$ 序列 0.225、0.45，$Q(s_1)$ 序列 0.5、0.75，一致。

## 5. 概率集成、MBPO、PETS 与 Dreamer 的思想

以下四个方法均为概念性介绍，具体实现细节与实验数字未核对。

### 5.1 概率集成与两类不确定性

$M$ 个概率模型各输出 $\mathcal N(\mu_m,\sigma_m^2)$。预测总方差分解为

$$\operatorname{Var}[s']=\underbrace{\frac1M\sum_m\sigma_m^2}_{\text{偶然（噪声，不随数据减少）}}+\underbrace{\frac1M\sum_m(\mu_m-\bar\mu)^2}_{\text{认知（模型分歧，可由数据减少）}}.$$

**手算（示例，非项目实测）**：5 个模型对同一输入的预测均值为 $1.02,\ 0.98,\ 1.10,\ 0.90,\ 1.00$：$\bar\mu=1.00$，分歧方差 $=(0.0004+0.0004+0.01+0.01+0)/5=0.00416$，标准差 $0.0645$（总体标准差，Python 复核）。模型分歧大的位置通常是训练数据稀少的位置，这个信号可用于"不要信这里的想象"。

### 5.2 PETS 与 MBPO

- **PETS（概率集成 + 轨迹采样 + 采样 MPC）**思想：用概率集成模型，把多个粒子分别在不同集成成员里传播，估计轨迹回报的期望，再用 CEM 等采样优化器选动作序列，滚动时域执行第一步。它没有显式策略网络，推理时在线规划。
- **MBPO（模型 + 模型无关 RL）**思想：真实数据存入真实缓冲区；每隔一段时间，从真实缓冲区取状态，用模型向前只展开 $k$ 步（短）生成"合成转移"放入模型缓冲区；SAC 同时用两类数据训练。核心结论是：**短回放使模型误差有界，同时放大了数据量**。

数据放大的算术（示例）：真实缓冲区 1000 条，每条状态出发回放 5 步，则合成 5000 条，样本量是真实的 5 倍，但每条合成数据的误差只经历 ≤5 步累积。

### 5.3 Dreamer 的思想

Dreamer 类方法（概念性介绍）面对图像观测：

1. 学一个**世界模型**：把图像编码为潜状态（同时含确定性循环部分与随机部分，常称 RSSM），学习潜空间动力学、奖励预测和重构；
2. 在潜空间里"想象"长度 $H\approx15$ 的轨迹（$H$ 取值为概念性示例，各版本不同，未核对）；
3. 用想象的回报训练 actor（对想象轨迹反向传播）与 critic（价值）。

它把"学习环境"与"学习策略"分开，策略训练完全发生在想象中，真实环境只提供数据来训练世界模型。更多世界模型内容见 [世界模型与具身基础模型综述](./14_世界模型与具身基础模型综述.md)。

## 6. 与 MPC 结合：随机采样 MPC、终端价值

### 6.1 学习的模型 + 采样优化

经典 MPC（见 [MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)）用解析线性/运动学模型，把问题化成 QP，由求解器给出解。如果模型是神经网络，目标函数不再是凸二次，QP 求解器不能直接用，常用**采样式优化**：

1. 采样 $N$ 条动作序列 $\{u^{(i)}_{0:H-1}\}$；
2. 用学到的模型向前推演，计算每条序列的总代价 $S^{(i)}$；
3. 依据代价更新采样分布或加权平均；
4. 执行第一步，下一周期重新规划（滚动时域）。

**CEM（交叉熵方法）**：取代价最低的 $n_e$ 条"精英"序列，用其均值与标准差更新高斯采样分布，迭代数次。

**MPPI（模型预测路径积分）**：用指数加权平均，温度 $\lambda$ 越小，越只看最好的样本：

$$w^{(i)}=\frac{\exp\!\big(-(S^{(i)}-S_{\min})/\lambda\big)}{\sum_j\exp\!\big(-(S^{(j)}-S_{\min})/\lambda\big)},\qquad u^{\text{new}}=\sum_iw^{(i)}u^{(i)}.$$

（减去 $S_{\min}$ 是数值稳定性技巧，不改变权重比。）

**手算（示例，非项目实测）**：4 条采样的第一步动作 $u=(0.5,-0.2,1.0,0.3)$，对应总代价 $S=(4.0,2.0,3.0,1.0)$。

- $\lambda=1$：未归一化权重 $e^{-3},e^{-1},e^{-2},e^{0}=0.0498,0.3679,0.1353,1$，和 $1.5530$，归一化 $w=(0.0321,0.2369,0.0871,0.6439)$，$u^{\text{new}}=0.0321\times0.5+0.2369\times(-0.2)+0.0871\times1.0+0.6439\times0.3=0.2490$；
- $\lambda=0.5$：权重变为 $(0.0021,0.1171,0.0158,0.8650)$，$u^{\text{new}}=0.2530$，更贴近最优样本的 $0.3$。

CEM 对同样的 6 条样本（$u=0.5,-0.2,1.0,0.3,0.8,-0.5$；代价 $4.0,2.0,3.0,1.0,2.5,5.0$）取最优 3 条（代价 1.0、2.0、2.5）：精英动作 $0.3,-0.2,0.8$，新均值 $0.3$，新标准差 $0.408$（总体标准差，Python 复核）。

采样式 MPC 的优点是对模型形式无要求、可处理非光滑代价；缺点是计算量大（样本数 × 视野 × 模型前向）、解不如 QP 精确，需要 GPU 并行才能满足实时。

### 6.2 终端价值：短视野逼近长视野

MPC 的代价在 $H$ 步之后被截断。若在终点加一个**终端代价** $V_f(x_H)$，且 $V_f$ 恰好等于无限视野下的最优价值函数，则 $H=1$ 的 MPC 也等价于无限视野最优控制（动态规划的 Bellman 原理）。学习的做法是：用 RL（TD 学习）或离线数据去拟合价值函数 $V_\phi$，把它放在 MPC 的终点。这样"短视野规划"+"长远价值"互补：规划用模型处理近期细节，价值函数提供远期信息。

**完整手算（示例，非项目实测，Python 复核）**：标量系统 $x_{k+1}=x_k+u_k$，阶段代价 $x_k^2+0.1u_k^2$，初始 $x_0=2$。动态规划（Riccati 的标量形式）：若 $V(x)=Px^2$，则一步最优 $u=-\dfrac{P}{0.1+P}x$，新的价值系数 $P'=1+\dfrac{0.1P}{0.1+P}$。

- 无限视野：迭代到不动点，$P_\infty=1.0916$，反馈增益 $K=\dfrac{P_\infty}{0.1+P_\infty}=0.9161$，$u_0=-0.9161\times2=-1.832$；
- 视野 $H=1$ 且无终端代价（$P_f=0$）：$K=0$，$u_0=0$。因为只看一步，下一步状态的代价不在代价里，规划器认为"不动最省"——典型的短视；
- $H=1$ 且终端代价 $5x^2$（粗略的价值估计）：$K=5/5.1=0.9804$，$u_0=-1.961$，接近最优；
- $H=1$ 且终端代价取 $P_\infty x^2$：$u_0=-1.832$，与无限视野完全一致；
- $H=3$、终端 0：$u_0=-1.8321$（视野稍长即已接近最优）。

这解释了为什么 [P5 的 MPC 轨迹跟踪](../../01_项目课程/P5_车式移动机器人自主导航/10_MPC轨迹跟踪.md) 需要合适的预测视野与终端处理，也解释了学习价值函数的意义。

### 6.3 学习的位置：三种结合方式

| 方式 | 学什么 | 保留什么 | 风险 |
|---|---|---|---|
| 学动力学（残差） | $g_\theta$ 补偿未建模项 | 解析模型 + 约束 + 求解器 | 残差不确定处需保守 |
| 学价值 / 终端代价 | $V_\phi$ 代替终端项 | 模型与短视野规划 | 价值误差被规划器利用 |
| 学参数 / 代价权重 | Q、R 或摩擦系数的自适应 | 控制结构 | 在线适应的稳定性 |

安全约束依然交给优化器硬约束（例如执行器限幅、障碍距离），学习部分只改善目标与模型，而不是替代安全保障。这与 [P6 的安全执行链](../../01_项目课程/P6_深开鸿M-Robots平台软件/08_安全执行链.md)（策略检查先于执行）思想一致。

## 7. 离线 RL：分布偏移与外推误差

### 7.1 问题

离线 RL 的设定：给定固定数据集 $\mathcal D$（由某个行为策略 $\pi_\beta$ 采集），**不再与环境交互**，要从中学出尽可能好的策略。场景：历史遥操作日志、车队运行记录、示教数据（见 [模仿学习](./10_模仿学习_行为克隆_DAgger_扩散策略.md)）。

为什么不能直接把 SAC 拿来用？因为 Q 学习的目标含 $\max_{a'}Q(s',a')$：

$$y=r+\gamma\max_{a'}Q(s',a').$$

数据集里没出现过的动作 $a'$（分布外，OOD）的 $Q$ 值没有数据校正，神经网络的外推可能随意偏高；$\max$ 会专门挑中这些被高估的动作；高估又沿着 Bellman 递推传播并放大——**外推误差**（extrapolation error）。因为不能与环境交互，没有机会发现"其实不好"，错误无法纠正。在线 RL 则可以去试一试、得到真实低回报而纠正。

### 7.2 离线 RL 的统一思想：悲观 / 保守

要么限制策略接近数据（支持集约束 / KL 约束），要么直接压低 OOD 动作的 $Q$ 值。形式上都是"对没有证据的东西保持怀疑"。

## 8. 离线 RL 的三类方法与手算（CQL、IQL）

### 8.1 三类方法概述

| 类别 | 思想 | 代表（概念性介绍，细节未核对） |
|---|---|---|
| 策略约束 | 学的策略只能选数据里见过的动作附近 | BCQ、BEAR；TD3+BC（在策略损失里加行为克隆项） |
| 价值保守 | 把 OOD 动作的 $Q$ 压低 | CQL |
| 隐式 / 不查询 OOD | 只用数据中的动作做备份，用期望分位逼近 max | IQL、AWR 类 |

### 8.2 TD3+BC 的目标（概念性）

策略损失把 Q 的最大化与行为克隆项相加：$\max_\pi\ \lambda\,Q(s,\pi(s))-(\pi(s)-a)^2$，其中 $\lambda=\alpha/\overline{|Q|}$ 用 Q 的平均幅度归一化。BC 项把策略拉回数据，Q 项推动提高。简单、稳定，常作基线。

### 8.3 CQL 的保守惩罚

CQL 在标准 TD 误差之外增加一项（离散动作形式）：

$$\mathcal L_{\text{CQL}}=\alpha\,\mathbb E_{s\sim\mathcal D}\Big[\log\sum_a\exp Q(s,a)-\mathbb E_{a\sim\pi_\beta}Q(s,a)\Big]+\mathcal L_{\text{TD}}.$$

第一项 $\log\sum\exp$ 是 $\max_aQ$ 的平滑近似，第二项是数据动作的 $Q$ 均值。最小化它们的差，就是"压低所有动作的 Q，抬高数据中出现的动作的 Q"。

**手算（示例，非项目实测）**：某状态有 3 个离散动作，数据里出现的是动作 0。当前 $Q=(1.0,\ 3.0,\ 1.2)$，动作 1 是被外推高估的 OOD 动作。

- $\log\sum\exp=\ln(e^{1}+e^{3}+e^{1.2})=\ln(2.7183+20.0855+3.3201)=\ln26.124=3.2629$；
- 惩罚 $=3.2629-Q(s,a_0)=3.2629-1.0=2.2629$（很大，且梯度主要压低动作 1：softmax 权重 $(0.104,\ 0.769,\ 0.127)$）；
- 经过若干次最小化，$Q=(1.0,\ 0.2,\ 0.4)$ 时惩罚降为 $\ln(e^{1}+e^{0.2}+e^{0.4})-1=\ln(2.7183+1.2214+1.4918)-1=1.6922-1=0.6922$，OOD 动作不再被偏好。

Python 复核：3.2629、2.2629、0.6922 一致。系数 $\alpha$ 过大会过于悲观，把 Q 压得太低而得到过于保守的策略；过小则不起作用。

### 8.4 IQL 的期望分位回归

IQL 的目标是**不去查询** OOD 动作：在 $V(s)$ 的拟合里，用非对称平方损失（期望分位损失）逼近 $Q(s,a)$ 在数据动作上的上分位：

$$\mathcal L_V=\mathbb E\big[\,\lvert\tau-\mathbb 1(u<0)\rvert\;u^2\,\big],\quad u=Q(s,a)-V(s).$$

$\tau=0.5$ 时是普通均方回归，得到均值；$\tau\to1$ 时，$V$ 逼近数据中 $Q$ 的最大值，而这个最大值只由数据里的动作产生，没有任何 OOD 动作参与，因此避开了外推误差。

**手算（示例，非项目实测）**：数据里某状态的五个 $Q$ 样本为 $(1,2,3,4,10)$，求最小化期望分位损失的 $V$：

| $\tau$ | $V^\star$ | 含义 |
|---|---|---|
| 0.5 | 4.000 | 样本均值 |
| 0.7 | 5.263 | 向较大值倾斜 |
| 0.9 | 7.692 | 更接近最大值 |

（Python 对 $V$ 做一维网格搜索复核。）这就是"用数据内的最大值来近似 $\max$"。随后用优势 $A=Q-V$ 做加权行为克隆，得到策略，全程不需要策略采样 OOD 动作。

### 8.5 评估困难

离线 RL 的另一个难点是**无法可靠评估**：没有环境可测，只能用离线估计（重要性采样、拟合 Q 评估），方差大或有偏。工程上常用保守做法：先在仿真或受限的真机上做小规模验证，上线前设安全包络（动作限幅、回退到经典控制器）。

## 9. 选择与组合：RL、MPC、模仿学习

| 场景 | 推荐 | 理由 |
|---|---|---|
| 模型清楚、有硬约束 | MPC / LQR | 可验证、可解释 |
| 接触丰富、难建模、有仿真器 | 在线 RL（PPO/SAC）+ 域随机化 | 仿真样本便宜 |
| 真机样本昂贵 | 基于模型 RL（残差模型 + 短回放） | 少样本 |
| 有大量历史数据、不能再试 | 离线 RL（IQL/CQL），先模仿学习做初始化 | 无交互 |
| 有专家示教 | 行为克隆 / 扩散策略 → 再用 RL 微调 | 先学到"像样"的行为 |
| 需要保证安全 | RL 作为上层 + MPC/CBF 作为安全层 | 学习提升性能，约束仍严格 |

常见组合流水线：示教数据 → 行为克隆预训练 → 离线 RL 改善 → 少量真机在线微调 → 安全层（限幅、力矩、速度监控）。

## 10. 在七个项目里怎么用（含诚实边界）

七个项目的 CV 表述均没有使用基于模型的 RL 或离线 RL。以下是可以在面试里诚实讲的联系：

| 项目 | 联系 | 性质 |
|---|---|---|
| P5 [MPC 轨迹跟踪](../../01_项目课程/P5_车式移动机器人自主导航/10_MPC轨迹跟踪.md) | 项目做的是基于车辆运动学（解析）模型的滚动时域优化。本章 6.1–6.3 的"采样 MPC"和"终端价值"是把同样的框架换成学习的模型 / 价值 | MPC 本身是项目所做；学习部分为延伸，非项目所做 |
| P1 [LQR 设计与分析](../../01_项目课程/P1_AIRS球形机器人平衡控制/05_LQR设计与分析.md) | LQR 的 Riccati 解就是"无限视野价值函数 $V=x^\top Px$"，6.2 节的标量例子是其特例；这是理解"终端价值 = 无限视野最优"的最好桥梁 | 数学联系 |
| P7 [LQR 原理与增益调度](../../01_项目课程/P7_可变腿长轮腿机器人/05_LQR原理与增益调度.md) | 增益调度按腿长切换 LQR，可看作"在不同工作点用不同的局部价值函数" | 类比；项目所做是增益调度，不是 RL |
| P2 [主从跟踪与 MIT 电机接口](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/07_主从跟踪与MIT电机接口.md) | 遥操作产生的主从轨迹数据可以成为模仿学习/离线 RL 的数据来源；项目本身是教学示教，没有训练策略 | 延伸，非项目所做 |
| P1/P7 调试与参数整定 | 在线调参可视为"人工的模型无关优化"；系统辨识是"学模型"的经典做法，见 [系统辨识与参数整定](../05_控制理论/14_系统辨识与参数整定.md) | 经典对应 |
| P6 [安全执行链](../../01_项目课程/P6_深开鸿M-Robots平台软件/08_安全执行链.md) | 学习得到的动作（包括 RL 策略）同样要经过提案—策略检查—执行—反馈的链，保守层独立于学习层 | 项目理念与 6.3 一致 |

> **面试要点**：当面试官问"你有没有用过强化学习"，诚实回答是：项目里用的是 PID、LQR、VMC、MPC 这类基于模型的经典控制，没有训练 RL 策略；但你能解释 LQR 的价值函数形式，和学习的价值函数在 MPC 终端项上的作用同构，并知道基于模型和离线 RL 的取舍。这样既不夸大，又展示了桥接能力。

## 11. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 想象里回报很高，真机一塌糊涂 | 策略利用模型漏洞 | 集成 + 分歧惩罚；缩短回放视野；在真实数据上定期评估 |
| 多步预测在几步后发散 | 单步损失训练 + 对象不稳定 | 多步损失、预测增量、限幅、缩短视野 |
| 模型只在训练分布内准确 | 数据覆盖窄 | 增加探索数据；加入随机扰动；从多初值采集 |
| 采样式 MPC 输出抖动 | 采样噪声、更新量大 | 增加样本数、动作低通滤波、热启动上一周期解 |
| MPPI 温度 $\lambda$ 设不当 | 过小退化为取最优样本，过大趋于平均 | 以代价尺度归一化后调节，观察有效样本数 |
| 离线 RL 的 Q 值爆炸 | OOD 动作高估 | 保守惩罚（CQL）、策略约束（BC 项）、更小学习率 |
| 离线 RL 效果不如行为克隆 | 数据质量高且覆盖窄时 BC 已很强；$\alpha$ 过大 | 先用 BC 做基线，再比较 |
| 离线评估和真机结果不一致 | 评估估计方差大 | 小范围真机验证，设置安全包络 |
| 用 RL 取代安全约束 | 把安全寄托在奖励惩罚上 | 约束由控制层硬保证，RL 只做性能优化 |

## 12. 面试怎么问、怎么答

**Q1. Compare model-based and model-free reinforcement learning. When would you choose each?**

> Model-free methods like PPO or SAC learn a policy or value function directly from interaction, so they are simple and have a high asymptotic performance, but they need a lot of samples. Model-based methods learn a dynamics model and use it for planning or for generating synthetic data, so they need far fewer real samples, but their performance is limited by the model error, and the policy can exploit that error. When real data is expensive, like on a physical robot, I would lean on a model-based approach with a physics prior and short rollouts; when I have a fast simulator, a model-free method with domain randomization is simpler.

中文：模型无关方法直接从交互中学习，简单、渐近性能高，但样本量大；基于模型方法学动力学再规划或生成合成数据，真实样本少，但受模型误差限制且策略会利用误差。真机数据昂贵时倾向基于模型 + 物理先验 + 短回放；有快速仿真时，模型无关 + 域随机化更简单。

*Follow-up: What is compounding model error and how do you handle it?*

> If the one-step error is epsilon and the dynamics amplify errors by a factor L per step, the k-step error grows like epsilon times L to the k minus one over L minus one, so exponentially for unstable systems. I handle it with short rollouts started from real states, ensembles to detect uncertain regions, multi-step training losses, and receding-horizon replanning so the controller keeps correcting itself.

中文：单步误差 $\varepsilon$，每步放大 $L$，$k$ 步误差 $\approx\varepsilon(L^k-1)/(L-1)$，不稳定系统指数增长；对策：从真实状态出发的短回放、集成检测不确定区域、多步损失、滚动时域重规划持续纠偏。

**Q2. Why does ordinary off-policy RL fail on a fixed offline dataset, and what does CQL do about it?**

> The Bellman target takes a max over actions at the next state, including actions that never appear in the dataset. The Q-function extrapolates arbitrarily on those out-of-distribution actions, the max picks the overestimated ones, and the error propagates through bootstrapping. In online RL the agent would try the action and correct the estimate, but offline it cannot. CQL adds a regularizer that pushes down the Q-values of all actions through a log-sum-exp term and pushes up the Q-values of the dataset actions, so the learned Q is a conservative lower bound.

中文：Bellman 目标对下一状态做 max，包含数据里没有的动作；Q 在这些 OOD 动作上任意外推，max 挑中被高估的，再沿自举传播。在线 RL 可以尝试并纠正，离线不行。CQL 加正则：用 log-sum-exp 压低所有动作的 Q、抬高数据动作的 Q，得到保守下界。

*Follow-up: What is the difference between CQL and IQL?*

> CQL penalizes out-of-distribution Q-values directly. IQL avoids querying out-of-distribution actions altogether: it fits the value function with an expectile regression on Q-values of dataset actions, which approximates the maximum over in-support actions, and then extracts the policy with advantage-weighted regression. IQL is usually simpler and more stable to tune.

中文：CQL 直接惩罚 OOD 的 Q；IQL 完全不查询 OOD 动作，用期望分位回归拟合数据动作上的 Q 来近似支持集内的最大值，再用优势加权回归提取策略；IQL 通常更简单稳定。

**Q3. How would you combine learning with MPC on a robot?**

> I would keep the MPC structure and constraints, because they give safety and interpretability, and learn only the parts that are hard to model. One option is a learned residual on top of the physics model, with an ensemble or Gaussian process to quantify where the residual is unreliable. Another is learning a value function to use as the terminal cost, so a short horizon behaves like a long one. If the learned model is a neural network, I would use a sampling-based optimizer like MPPI or CEM instead of a QP solver. The hard constraints stay in the controller so that the learned part cannot violate them.

中文：保留 MPC 结构与约束（安全与可解释），只学难建模部分：在物理模型上学残差并用集成/GP 量化不可靠区；或学价值函数作终端代价让短视野逼近长视野；学习模型是神经网络时用 MPPI/CEM 代替 QP；硬约束留在控制器里。

**Q4. Why can a short MPC horizon work well if the terminal cost is chosen properly?**

> By Bellman's principle of optimality, if the terminal cost equals the optimal cost-to-go, then a one-step lookahead already gives the optimal action. In LQR the cost-to-go is the quadratic form with the Riccati solution. In a scalar example I worked out, a horizon of one with the Riccati value as terminal cost gives exactly the infinite-horizon control, while a horizon of one with no terminal cost gives zero control and is myopic.

中文：由 Bellman 最优性原理，终端代价等于最优代价到达时，一步前瞻即最优；LQR 中最优代价到达是 Riccati 解的二次型。标量例子里，视野 1 加 Riccati 终端项恰好等于无限视野控制，而无终端项则输出 0，是短视的。

**Q5. Have you used reinforcement learning or learned models in your projects?**（项目对照）

> Not in the work on my CV. My balance and navigation projects used model-based classical control: cascaded PID, LQR analysis, VMC, and MPC on a kinematic model. But I understand how the same ideas extend: the LQR cost-to-go is a value function, MPC with a learned terminal value is a bridge to RL, and I know why offline RL needs conservatism. That is the direction I want to learn more about in graduate study.

中文：简历中的工作没有用 RL。平衡和导航项目用基于模型的经典控制：串级 PID、LQR 分析、VMC 和运动学模型上的 MPC。但我理解思路的延伸：LQR 的代价到达就是价值函数，带学习终端价值的 MPC 是通向 RL 的桥梁，也知道离线 RL 为何需要保守；这是我希望在研究生阶段深入的方向。

## 13. 本章要点速记

1. 基于模型 RL：学 $f_\theta$，再规划或生成合成数据；样本省，但模型误差会被策略利用。
2. 误差累积：$e_k\lesssim\varepsilon(L^k-1)/(L-1)$；不稳定系统 1.9% 的一步误差十步后可到约 21%。
3. 对策：短回放、集成分歧惩罚、多步损失、滚动时域、物理先验 + 残差。
4. Dyna：真实经验 + 模型回放，价值传播更快（手算 $Q(s_0)$：0 → 0.45）。
5. 概率集成：总方差 = 偶然（平均方差）+ 认知（均值分歧）。
6. MBPO：短回放增数据；PETS：集成 + 采样 MPC；Dreamer：潜空间想象。
7. 采样 MPC：CEM 用精英均值/方差，MPPI 用 $\exp(-S/\lambda)$ 加权。
8. 终端价值：终端代价 = 最优代价到达时，一步前瞻即最优；LQR 的 $P$ 即该价值。
9. 离线 RL：OOD 动作被 max 挑中导致外推误差；对策是保守（CQL）、约束（BC 项）、不查询 OOD（IQL）。
10. 安全约束留在控制器，学习只提升性能；本章学习方法对七个项目是延伸，非项目所做。

## 14. 自测题

1. 3.1 节的误差上界中 $L=1.05$、单步误差 $\varepsilon$，3 步误差上界是 $\varepsilon$ 的多少倍？（$(1.05^3-1)/(1.05-1)=0.157625/0.05=3.1525$ 倍。）
2. Dyna-Q 中 $\alpha=0.5$，$\gamma=0.9$，$Q(s_1)=0.75$，回放 $(s_0,a,0,s_1)$ 且 $Q(s_0)=0.45$，更新后 $Q(s_0)$？（目标 $0.9\times0.75=0.675$；$0.45+0.5\times(0.675-0.45)=0.5625$。）
3. MPPI 中 $\lambda\to\infty$ 与 $\lambda\to0$ 分别退化成什么？（$\lambda\to\infty$ 权重趋于均匀，取样本平均；$\lambda\to0$ 只保留最低代价样本。）
4. 为什么离线 RL 的 Q 值估计容易高估而在线 RL 较少这个问题？（离线无法尝试 OOD 动作来校正；max 会选中被外推高估的动作。）
5. 期望分位回归中 $\tau=0.9$ 对"高于估计值的样本"与"低于估计值的样本"的损失权重各是多少？（高于：0.9；低于：0.1。）

## 15. 相关链接

- 上一章：[概率图模型、贝叶斯方法与高斯过程](./12_概率图模型_贝叶斯方法_高斯过程.md)；下一章：[世界模型与具身基础模型综述](./14_世界模型与具身基础模型综述.md)
- 前置：[强化学习进阶：策略梯度、GAE、PPO、SAC](./09_强化学习进阶_策略梯度_PPO_SAC.md)、[强化学习与具身智能入门](./06_强化学习与具身智能入门.md)
- 控制侧：[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)、[LQR 最优控制](../05_控制理论/04_LQR最优控制.md)
- 八股速查：[强化学习八股](../../03_八股知识/02_人工智能/05_强化学习八股.md)
- 模块说明：[10_机器学习与深度学习 README](./README.md)
