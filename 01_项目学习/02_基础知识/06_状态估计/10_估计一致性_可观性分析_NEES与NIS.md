# 估计一致性、可观性分析、NEES 与 NIS

> **一句话定位**：一个好的滤波器不只是“估得准”，还要“知道自己有多准”——它报告的协方差必须与它的真实误差统计相符，这叫**一致性**（consistency）。而有些状态方向无论多聪明的滤波器都估不出来（比如只有速度测量时的绝对位置、只有重力时的偏航），这叫**不可观**。本章把两件事连起来：先用可观性判断“能不能估”，再用 NEES/NIS 检验“估得诚不诚实”，最后给出一套调参排错流程。

**本章学完你能：**

- 用可观性矩阵秩判断线性系统，手算常速模型在“只测位置”与“只测速度”下的可观性，并解释“只测速度时位置协方差线性增长”的数值现象；
- 解释非线性系统的局部可观性与不可观方向，手算“机器人—路标”相对测量中全局平移与旋转的 3 维不可观子空间；
- 说明 EKF 为什么会因线性化点偏离真值而“虚构”出对不可观方向的信息，用 Python 给出数值（信息从 0 变成 0.02/次）；
- 区分 NEES（需要真值）与 NIS（不需要真值）、单次运行与多次运行的集合平均，并推导 $\chi^2$ 置信区间（含 Wilson–Hilferty 近似）；
- 阅读蒙特卡洛实验表：$Q$ 偏小、一致、偏大时 ANEES/ANIS 与新息自相关的变化；
- 执行一套“从静态数据到实机新息”的调参排错清单；
- 用英文回答 "What is filter consistency / how do you tune Q and R / what makes EKF-SLAM inconsistent" 等问题。

**前置知识**：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)（NEES/NIS 初步与 Q/R 调参，本章是其深化）、[状态空间：可控可观、稳定性](../05_控制理论/03_状态空间_可控可观_稳定性.md)（可观性矩阵）、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)（$\chi^2$ 分布）、[ESKF 与流形上的滤波](./09_误差状态卡尔曼ESKF与流形上的滤波.md)（重力不观测偏航）、[随机信号…Allan 方差](../04_信号与系统/04_随机信号_自相关_功率谱_白噪声与Allan方差.md)（白性检验）。

## 目录

1. 问题：协方差可以"说谎"
2. 符号
3. 可观性：线性系统
4. 手算：常速模型的可观性与协方差
5. 非线性系统与不可观方向
6. EKF 为什么会变得不一致
7. 一致性检验：NEES、NIS 与置信区间
8. 蒙特卡洛实验：Q 偏小、一致、偏大
9. 其他不一致来源
10. 调参与排错流程
11. 在本项目里怎么用
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 问题：协方差可以"说谎"

假设你写了一个滤波器，输出估计 $\hat x$ 和协方差 $P$。机器人的下游模块（规划、控制、数据关联、门控）会把 $P$ 当成“我对 $\hat x$ 有多确定”的凭据：门控用它判断测量是否异常，规划用它留安全余量，融合用它给权重。如果 $P$ 太小（过度自信，optimistic），下游会过度信任一个其实不准的状态——典型症状是“估计很平滑，但越走越偏，直到突然出大错”；如果 $P$ 太大（过度保守，pessimistic），下游会不必要地拒绝好数据，响应迟钝。

所以我们不仅要检查 $\lVert x-\hat x\rVert$ 小不小，还要检查 $P$ 与实际误差统计是否一致。这就是一致性。

另一个相关的问题：有些量**本来就估不出来**。如果一个方向在数学上不可观，滤波器的正确行为是让该方向的协方差保持（或增长）不变；如果滤波器反而让它的协方差变小，就说明它“虚构”了信息，这是最常见的不一致来源之一。

## 2. 符号

| 符号 | 含义 |
|---|---|
| $x_k,\hat x_k,P_k$ | 状态真值、估计、估计协方差 |
| $e_k=x_k-\hat x_k$ | 估计误差 |
| $\nu_k=z_k-h(\hat x_k^-)$ | 新息（innovation） |
| $S_k$ | 新息协方差 |
| $\epsilon_k=e_k^\top P_k^{-1}e_k$ | NEES（单次运行、单时刻） |
| $\eta_k=\nu_k^\top S_k^{-1}\nu_k$ | NIS |
| $n,m$ | 状态维数、测量维数 |
| $N$ | 蒙特卡洛运行次数 |
| $\mathcal O$ | 可观性矩阵 |
| $\mathcal N(\cdot)$ | 零空间 |
| $\chi^2_\nu$ | 自由度 $\nu$ 的卡方分布 |

## 3. 可观性：线性系统

### 3.1 定义

线性时不变系统 $x_{k+1}=Fx_k$，$z_k=Hx_k$。若由输出序列 $z_0,\dots,z_{n-1}$ 能唯一确定初始状态 $x_0$，则称系统可观。

### 3.2 秩判据

可观性矩阵

$$\mathcal O=\begin{bmatrix}H\\HF\\\vdots\\HF^{n-1}\end{bmatrix},$$

系统可观 ⇔ $\mathrm{rank}(\mathcal O)=n$。其零空间 $\mathcal N(\mathcal O)$ 就是**不可观子空间**：初始状态沿这些方向的任何变化都不会改变输出。

### 3.3 与协方差的关系

对不可观方向，没有任何测量能降低它的不确定性。因此：

- 沿不可观方向，滤波器的协方差**不会因测量而减小**；
- 若系统在该方向上动力学是不稳定的或有过程噪声，协方差会**无界增长**；
- 对可观方向，在满足可检测/稳定性条件下，协方差收敛到有界稳态。

于是，**协方差的"形状"本身就是可观性的诊断工具**：某些方向的方差持续增长，通常对应不可观（或弱可观）的量。

## 4. 手算：常速模型的可观性与协方差

模型：位置 $p$、速度 $v$，$F=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}$（$\Delta t=1$ s）。

### 4.1 秩判据

- **只测位置** $H=[1\ 0]$：$\mathcal O=\begin{bmatrix}1&0\\1&1\end{bmatrix}$，秩 2，可观（奇异值 $1.618,\ 0.618$）；
- **只测速度** $H=[0\ 1]$：$\mathcal O=\begin{bmatrix}0&1\\0&1\end{bmatrix}$，秩 1，**位置不可观**（奇异值 $1.414,\ 0$）。（Python 复核。）

### 4.2 协方差数值

取过程噪声 $Q=q\begin{bmatrix}0.25&0.5\\0.5&1\end{bmatrix}$（$q=0.01$，白噪声加速度模型的简化），$R=0.01$，初值 $P_0=I$。滤波迭代（Python）：

| 步数 | 只测位置：$P_{pp}$ | 只测位置：$P_{vv}$ | 只测速度：$P_{pp}$ | 只测速度：$P_{vv}$ |
|---|---|---|---|---|
| 1 | 0.00995 | 0.508 | 1.012 | 0.0099 |
| 10 | 0.00750 | 0.0100 | 1.105 | 0.00618 |
| 50 | 0.00750 | 0.0100 | 1.505 | 0.00618 |
| 100 | 0.00750 | 0.0100 | **2.005** | 0.00618 |

读法：

- 只测位置：两个方差都很快收敛到稳态（$0.0075,\ 0.0100$）；
- 只测速度：$P_{vv}$ 收敛，但 $P_{pp}$ **随时间线性增长**（每步约 $+0.01$，即速度估计的小误差被积分成位置不确定性，外加过程噪声）。滤波器非常诚实：它说“我越来越不知道自己在哪”。

这是一个重要的工程教训：**纯里程计（速度）类传感器无法约束绝对位置，协方差必然增长**；要得到有界误差，需要绝对位置类测量（GPS、地图匹配、AMCL 的激光匹配）。P5 里 AMCL 的作用，正是给 EKF 提供绝对位置的“锚”（见 [EKF 与 AMCL 协同](../../01_项目课程/P5_车式移动机器人自主导航/06_EKF与AMCL协同_TF分工与连续状态.md)）。

## 5. 非线性系统与不可观方向

### 5.1 局部可观性

对非线性系统 $z=h(x)$，在某个点附近做线性化，计算线性化的可观性矩阵的秩，可给出**局部可观性**的充分判断（严格说需要 Lie 导数条件，细节不展开）。把对时间的展开也包含进去，得到一系列雅可比 $H_k,H_{k+1}\Phi_{k+1,k},\dots$ 叠起来的矩阵，其零空间是（线性化意义下的）不可观子空间。

### 5.2 典型例子：位姿—路标的相对测量

状态：机器人位姿 $(x,y,\theta)$ 与一个路标位置 $(l_x,l_y)$，共 5 维。测量：路标在机器人坐标系下的位置

$$z=R(\theta)^\top\big([l_x,l_y]^\top-[x,y]^\top\big)\in\mathbb{R}^2.$$

这个测量只依赖**相对关系**。如果把整个系统（机器人和路标）一起平移 $(t_x,t_y)$ 或一起绕原点旋转，测量完全不变。所以有 3 维不可观方向（全局平移 2 维 + 全局旋转 1 维），即**整个相对系统的刚体运动自由度**。

**数值验证**：真实状态 $(x,y,\theta,l_x,l_y)=(1,2,0.5,4,3)$。测量雅可比（数值差分）

$$H=\begin{bmatrix}-0.8776&-0.4794&-0.5607&0.8776&0.4794\\0.4794&-0.8776&-3.1122&-0.4794&0.8776\end{bmatrix},$$

秩 2（5 维状态、2 个方程）。三个不可观方向为：

- 平移 $x$：$N_x=(1,0,0,1,0)$；
- 平移 $y$：$N_y=(0,1,0,0,1)$；
- 旋转：$N_r=(-y,\ x,\ 1,\ -l_y,\ l_x)=(-2,\ 1,\ 1,\ -3,\ 4)$。

Python 验证：$HN_x=HN_y=HN_r=0$（数值上 $10^{-16}$ 量级）。也就是说，任何仅靠这种相对观测的滤波器/SLAM 系统，**绝对位置和绝对朝向永远估不出来**，必须在初始时给一个先验（锚定），或引入绝对观测（GPS、已知地图）。这就是 SLAM 中“规范自由度”（gauge freedom）的来源，位姿图优化需要固定第一个位姿（先验因子）也是同样的道理（见 [SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)）。

## 6. EKF 为什么会变得不一致

### 6.1 虚假的信息

EKF 的雅可比 $H$ 是在**估计值** $\hat x$ 处计算的，而不是在真值处。真值处，$HN=0$（不可观方向上没有信息）；估计值处，$H(\hat x)N\neq0$（一般），于是 EKF 会认为“沿 $N$ 的变化会改变测量”，从而**用这一条本来无信息的测量去减小 $N$ 方向的协方差**。这是 EKF-SLAM 中偏航协方差随时间下降而实际误差并未下降的原因：过度自信、不一致（Huang 与 Mourikis 等人的分析给出了这个结论，具体文献细节未核对）。

**数值**：令线性化点相对真值偏离 $(0.05,\ -0.04,\ 0.03,\ 0.06,\ -0.05)$（位置误差几厘米量级、角度误差 1.7°——对一个 EKF 非常普通）。在该点计算的 $H_e$ 仍然是秩 2，但

$$H_eN_r=(-0.0137,\ -0.0036)\neq0,$$

于是沿旋转方向的“信息” $N_r^\top H_e^\top R_m^{-1}H_eN_r=0.0200$（取 $R_m=0.1^2I$），而真值处是 $\approx10^{-16}$。**每次测量虚构 0.02 的信息**；1000 次测量后虚构信息累计到约 20，对应的偏航标准差 $\sim1/\sqrt{20}=0.22$ rad（约 12.8°），本该是“完全未知”的方向被滤波器认为“有 0.22 rad 的精度”。

### 6.2 解决思路

1. **FEJ（First-Estimate Jacobian）**：每个状态变量的雅可比始终在它**第一次被估计的值**处计算，不随后续估计变化，从而让不可观方向在线性化后保持不可观。这是一致性 EKF-SLAM 和滑动窗口 VIO（也包括上一章边缘化先验）的常用做法；
2. **可观性约束（OC-EKF）**：显式修改雅可比，使其与理论不可观子空间正交；
3. **不变 EKF / 合适的误差定义**：选取使不可观方向与状态无关的误差坐标（见 [ESKF](./09_误差状态卡尔曼ESKF与流形上的滤波.md)）；
4. **加先验锚定 + 保守的噪声设置**：例如固定初始位姿；
5. **迭代更新、延迟线性化**：让线性化点更接近真值（但不能消除问题的根源）。

## 7. 一致性检验：NEES、NIS 与置信区间

### 7.1 定义

若滤波器一致，且噪声是高斯，则

$$\epsilon_k=e_k^\top P_k^{-1}e_k\ \sim\ \chi^2_n,\qquad \eta_k=\nu_k^\top S_k^{-1}\nu_k\ \sim\ \chi^2_m.$$

- NEES（需要真值，仿真或带高精度参考的实验）检验**估计误差**的统计是否与 $P$ 相符；
- NIS（不需要真值，实机可用）检验**新息**的统计是否与 $S$ 相符。

### 7.2 多次运行的集合平均

对同一场景做 $N$ 次独立蒙特卡洛运行，在固定时刻 $k$ 取平均：

$$\bar\epsilon_k=\frac1N\sum_{i=1}^N\epsilon_k^{(i)},\qquad N\bar\epsilon_k\sim\chi^2_{Nn}.$$

因此 $\bar\epsilon_k$ 的 95% 区间是 $\big[\chi^2_{Nn}(0.025),\ \chi^2_{Nn}(0.975)\big]/N$。NIS 同理，自由度 $Nm$。

**为什么集合平均**：同一次运行中相邻时刻的 NEES/NIS 是相关的（估计误差有记忆），把它们当独立样本求平均会使真实方差大于 $\chi^2$ 理论值，误判。跨独立运行的平均才是独立的。

### 7.3 区间的计算（Wilson–Hilferty 近似）

没有统计表时，$\chi^2_\nu$ 的分位数可近似为 $\nu\big(1-\tfrac2{9\nu}+z\sqrt{\tfrac2{9\nu}}\big)^3$，$z=\pm1.96$ 对应 95%。**手算**：$N=1000$ 次运行，$n=2$，$\nu=2000$：$\tfrac2{9\nu}=1.11\times10^{-4}$，$\sqrt{\cdot}=0.01054$，下界 $\nu(1-1.11\times10^{-4}-0.02066)^3\approx1.8779\times10^3$，上界 $\approx2.1258\times10^3$。除以 $N$：ANEES 区间 $[1.878,\ 2.126]$（Python 复核）。NIS（$m=1$，$\nu=1000$）区间 $[0.914,\ 1.090]$。

### 7.4 解读规则

| 结果 | 含义 | 常见原因 |
|---|---|---|
| ANEES 高于上界 | 过度自信（optimistic） | $Q$ 或 $R$ 偏小；模型有偏；线性化不一致；信息重复使用 |
| ANEES 低于下界 | 过度保守（pessimistic） | $Q$ 或 $R$ 偏大；多余的鲁棒化 |
| ANIS 高于上界 | 新息比预期大 | 模型漏动态，$R/Q$ 偏小，异常值，延迟 |
| ANIS 低于下界 | 新息比预期小 | $S$ 偏大（$R$ 或 $P$ 偏大） |
| 区间内 | 与一致性假设相容（不是证明） | |

注意：落在区间内只说明"没发现不一致"，并不证明正确；落在区间外是有力的证据。

### 7.5 NIS 的补充：新息的白性与零均值

一致的滤波器的新息序列还应：（1）零均值；（2）时间上不相关（白）。一阶自相关系数 $\hat r[1]$ 应在 $\pm2/\sqrt{N_s}$ 内（$N_s$ 为样本数，见 [随机信号…白噪声](../04_信号与系统/04_随机信号_自相关_功率谱_白噪声与Allan方差.md)）。新息有显著正自相关，说明滤波器反应过慢（$Q$ 偏小）：误差一旦出现，会持续数步。

## 8. 蒙特卡洛实验：Q 偏小、一致、偏大

### 8.1 设置

被估系统：常速模型，$\Delta t=0.1$ s，真实过程噪声（加速度）强度 $q_{\text{true}}=0.5$，位置测量方差 $R=0.04$；初始真值按 $P_0=I$ 抽样。滤波器使用 $Q_f=s\cdot Q_{\text{true}}$，其中 $s$ 为“失配系数”（$s=1$ 即匹配）。Python 蒙特卡洛：$N=1000$ 次运行，在第 100 步取集合平均；ANEES 区间 $[1.878,\ 2.126]$，ANIS 区间 $[0.914,\ 1.090]$。

### 8.2 结果

| $Q$ 失配系数 $s$ | ANEES（$n=2$） | ANIS（$m=1$） | 判断 |
|---|---|---|---|
| 0.1 | **10.63** | **1.374** | 严重过度自信：ANEES 是理论值的 5 倍多 |
| 1.0 | 1.954 | 1.016 | 在区间内：一致 |
| 10 | **1.153** | 0.922 | 过度保守：ANEES 低于下界；ANIS 勉强在区间内 |

### 8.3 观察

1. **ANEES 对 $Q$ 失配远比 ANIS 敏感**：$s=0.1$ 时 ANEES 偏高 5.4 倍，ANIS 只偏高 35%。原因是 NIS 只看一步新息，而 $Q$ 偏小造成的累积误差在 NEES 里（通过 $P$ 偏小）被放大。这也说明：**只有 NIS（实机常常只有 NIS）时，对"偏保守"的失配不敏感**，需要结合其他诊断；
2. **偏保守比偏自信"便宜"**：$s=10$ 时 ANEES 只低到 1.15（理论 2），估计稍慢但没有崩；$s=0.1$ 则是协方差严重低估，下游门控会拒绝大量好数据；
3. **新息自相关**：时间平均版实验（50 次运行，每次 200 步，去掉前 20 步）中，新息的一阶自相关系数：$s=0.1$ 时 $+0.24$，$s=0.3$ 时 $+0.11$，$s=1$ 时 $+0.017$，$s=3$ 时 $-0.043$，$s=10$ 时 $-0.096$。**$Q$ 偏小 ⇒ 正相关，$Q$ 偏大 ⇒ 轻微负相关**，匹配时接近 0。这是不需要真值的诊断信号：看新息相关的符号和大小，就能知道往哪个方向调 $Q$。

## 9. 其他不一致来源

| 来源 | 症状 | 排查 |
|---|---|---|
| $R$ 偏小（相信传感器过多） | NIS 偏高，估计跟着测量噪声抖 | 静态数据重估 $R$ |
| 有色噪声当白噪声 | 新息自相关显著，协方差偏小 | 扩维建模偏置/相关噪声 |
| 时间戳错误/延迟 | 新息在运动时变大，静止时正常 | 对时、回放；加延迟补偿 |
| 坐标系/外参错误 | 新息有系统性偏置 | 标定外参，检查 TF |
| 同一信息被重复使用 | 协方差快速变小，实际误差不降 | 追踪信息来源，CI 融合 |
| 异常值 | NIS 偶发极大值 | 马氏距离门控，鲁棒核 |
| 线性化不一致 | 不可观方向协方差下降 | FEJ、OC-EKF、迭代 |
| 模型错误（打滑、轮径） | 新息偏置、方向性 | 增加状态（轮径、打滑因子），或增大 $Q$ |
| 数值问题 | 协方差非对称/非正定 | Joseph 形式、对称化、平方根滤波 |
| 采样周期变化 | $Q_d,R$ 未随 $\Delta t$ 缩放 | 由连续噪声密度重算 |

## 10. 调参与排错流程

一套顺序明确的清单（每一步的结果决定下一步）：

1. **无噪声仿真**：把噪声设为零，滤波器应该精确跟踪真值（误差 $\sim10^{-6}$）。做不到说明模型/雅可比/代码有错，不要调参；
2. **确定 $R$**：让机器人静止，记录传感器，计算样本方差（注意量纲与采样率）；或从数据手册的噪声密度换算（见随机信号章）；
3. **确定 $Q$ 的初值**：由物理量估算（加速度扰动、打滑、模型误差在一个周期内造成的状态变化），或由 IMU 噪声密度换算；
4. **仿真闭环**：在已知真值的仿真里跑蒙特卡洛，看 ANEES/ANIS 是否在区间内，不在则调整 $Q,R$ 的比例；
5. **实机上看 NIS**：时间平均 NIS（滑窗，比如 100 个样本）应接近 $m$；分段（直线、转弯、静止）分别看，不同工况可能需要不同的 $Q$；
6. **看新息的自相关与均值**：正相关 ⇒ 增大 $Q$（或补建模）；均值非零 ⇒ 存在偏置/外参/对时问题；
7. **检查不可观方向**：协方差随时间增长的方向是否与理论不可观一致？如果理论不可观的方向协方差减小了，检查线性化点和是否重复使用信息；
8. **压力测试**：突然加速、打滑、传感器丢失、断连后恢复，观察协方差是否随之合理变化（丢失传感器时协方差应增长）；
9. **记录**：把 $Q,R$ 取值、依据、验证指标写入调参记录。

**调参原则**：（1）一次只改一个量；（2）先改 $Q$ 再改 $R$（或相反），不要同时；（3）$Q/R$ 的比值决定增益，两者同时缩放不改变稳态增益，但改变报告的协方差；（4）宁可略保守；（5）任何调参之后重新运行一致性检验。

## 11. 在本项目里怎么用

> 说明：CV 的 P5 写到“处理由时序、坐标系和运动方向不一致引起的漂移和位姿跳变”，这些都属于本章“其他不一致来源”的具体症状。NEES/NIS 作为系统化的检验方法是延伸知识，面试时说成“我如何判断滤波器是否可信”。

- **P5 EKF/AMCL**：EKF 融合轮速里程计、IMU 与 AMCL 位姿；里程计只测速度，位置不可观，协方差会增长；AMCL 的绝对位姿测量把它拉回来。排查“位姿跳变”时：先看时间戳与坐标系（第 9 节前三行），再看新息是否在某个工况下系统性偏大（见 [P5 传感器与数据质量：时间、坐标与运动方向](../../01_项目课程/P5_车式移动机器人自主导航/03_传感器与数据质量_时间坐标与运动方向.md)、[P5 EKF 融合定位](../../01_项目课程/P5_车式移动机器人自主导航/04_EKF融合定位.md)、[P5 联调与故障案例](../../01_项目课程/P5_车式移动机器人自主导航/11_联调与故障案例.md)）。
- **P1 / P3 / P7 姿态**：重力不观测偏航；没有磁力计时偏航协方差应当增长而非收敛（见 [ESKF](./09_误差状态卡尔曼ESKF与流形上的滤波.md)）。如果你的姿态滤波器对偏航也给出很小的方差，那是不一致的信号。
- **P7 状态估计**：速度由轮速/关节速度估计，打滑和离地时测量不可信，等价于测量不一致，需要门控和切换（见 [P7 离地检测与接触切换](../../01_项目课程/P7_可变腿长轮腿机器人/09_离地检测与接触切换.md)）。
- **P6 状态管理**：多源状态的协方差/可信度字段必须诚实，否则下游任务层得到的“可信度”是错的（见 [机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)）。

## 12. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 协方差持续下降到很小，但位置越走越偏 | 过度自信：$Q/R$ 偏小，信息重复使用，不可观方向被虚假观测 | 增大 $Q,R$；检查信息来源；FEJ |
| 位置协方差持续增长 | 缺少绝对位置观测（可能是正常现象） | 加绝对测量，或接受 |
| NIS 平均总在 $m$ 的 2–3 倍 | 噪声欠估、模型漏动态、时间戳错误 | 重估 $R$，补模型，检查对时 |
| NEES 区间判断总是“偏高” | 把同一次运行的时间样本当独立样本 | 用多次运行的集合平均 |
| 调 $Q$ 无论怎么调都不一致 | 模型结构错误，不是参数问题 | 检查模型、坐标系、单位 |
| 新息均值不为零 | 偏置、外参、重力方向、时间偏移 | 估计偏置，标定外参，对时 |
| 滤波器更新后协方差出现负对角元 | 数值问题 | Joseph 形式、对称化 |
| 把不可观量的协方差强行"压小"以通过检验 | 违背物理 | 保持不可观方向的诚实增长 |
| 仿真通过检验，实机通不过 | 仿真噪声过于理想 | 用实机静态数据、真实延迟进行仿真 |

## 13. 面试怎么问、怎么答

**Q1. What does it mean for a filter to be consistent, and how do you test it?**

> A filter is consistent when its reported covariance matches its actual error statistics. With ground truth, I use the normalised estimation error squared and average it over many Monte Carlo runs at a fixed time; it should be chi-square distributed with n times N degrees of freedom divided by N. On a real robot where there is no truth, I use the normalised innovation squared, whose expected value is the measurement dimension, and check that the innovations are zero-mean and white. Being outside the confidence interval is strong evidence of a problem, being inside only means I found none.

中文：滤波器一致 = 报告的协方差与实际误差统计相符；有真值时用 NEES，在固定时刻对多次蒙特卡洛运行取平均，应服从 $\chi^2_{nN}/N$；实机无真值时用 NIS，期望等于测量维数，并检查新息零均值、白；区间外是有力证据，区间内只表示没发现问题。

**Q2. What is observability and how does it show up in the covariance?**

> A direction in the state space is unobservable if changing the state along it never changes any measurement. For a linear system I test the rank of the observability matrix. In the filter the covariance along an unobservable direction cannot shrink and often grows. For example a constant-velocity model with only velocity measurements has an unobservable position: in my simulation the position variance grew from 1 to 2 over 100 steps while the velocity variance converged. That is the filter being honest.

中文：沿某个状态方向变化不改变任何测量，则该方向不可观；线性系统检验可观性矩阵的秩；滤波器中不可观方向的协方差不会减小，常增长；例如常速模型只测速度时位置不可观，仿真中位置方差 100 步内从 1 增到 2，而速度方差收敛——滤波器是诚实的。

**Q3. Why is EKF-SLAM inconsistent?**

> The measurement Jacobians are evaluated at the current estimate, not at the truth. Pose and landmark relative measurements are invariant to a global translation and rotation, so those three directions are unobservable, but with Jacobians at the estimate the product of the Jacobian and the unobservable direction is no longer zero. The filter then gains spurious information about global yaw and shrinks its uncertainty even though nothing was learned. The fixes are first-estimate Jacobians, observability-constrained EKF, or formulating the problem so those directions stay unobservable.

中文：雅可比在当前估计而非真值处求值；位姿与路标的相对测量对全局平移和旋转不变，这 3 个方向不可观，但在估计点求值时雅可比与不可观方向之积不再为零；滤波器就对全局偏航获得虚假信息并缩小不确定性；对策：FEJ、可观性约束 EKF、或构造使这些方向保持不可观的表述。

**Q4. How do you tune Q and R in practice?**

> I get R from static data or the sensor noise density, converted to the filter's sample time. I estimate Q from physical disturbances or IMU noise. Then I check consistency: in simulation with Monte Carlo runs, and on the robot with the time-averaged NIS and the innovation autocorrelation. Positive autocorrelation and a high NIS mean Q is too small; negative autocorrelation and a low NIS mean it is too large. I change one parameter at a time and rerun the checks. In my experiment, ten times too small a Q gave an ANEES five times above its bound, while ten times too large only brought it slightly below.

中文：R 来自静态数据或噪声密度并按滤波周期换算；Q 由物理扰动或 IMU 噪声估计；然后做一致性检验：仿真中蒙特卡洛，实机上看时间平均 NIS 与新息自相关；正自相关 + NIS 偏高 ⇒ Q 太小，负自相关 + NIS 偏低 ⇒ Q 太大；一次只改一个参数并重新检验；实验中 Q 小 10 倍使 ANEES 超界约 5 倍，大 10 倍只略低于下界。

**Q5. Why is a pessimistic filter usually safer than an optimistic one?**

> An optimistic filter reports a small covariance while the real error is large, so gating rejects good measurements, the planner takes unsafe margins, and fusion over-weights a bad source. A pessimistic filter is slower and less accurate, but its uncertainty still bounds the true error, so downstream modules behave sensibly. That is why I would inflate noise slightly rather than shrink it, as long as I can still meet the accuracy requirement.

中文：自信的滤波器协方差小而真实误差大，导致门控拒绝好数据、规划留的余量不安全、融合过度信任坏来源；保守的滤波器更慢、精度略低，但不确定性仍能包住真实误差，下游模块行为合理；所以宁可略放大噪声，只要仍满足精度要求。

## 14. 本章要点速记

1. 一致性 = 报告的协方差与实际误差统计相符；过度自信比过度保守危险。
2. 线性可观性：$\mathrm{rank}\,\mathcal O=n$；不可观方向的协方差不减小，可能增长。
3. 常速模型：只测位置可观；只测速度时 $P_{pp}$ 线性增长（100 步 1→2）。
4. 位姿—路标相对测量：3 维不可观（全局平移 2 + 旋转 1），$HN=0$；SLAM 必须锚定。
5. EKF 在估计点线性化会虚构不可观方向的信息（示例每次 0.02，1000 次约 20 → 虚假精度 0.22 rad）。
6. 对策：FEJ、OC-EKF、不变误差、先验锚定。
7. NEES 需真值，NIS 不需；用多次运行的集合平均检验，区间 $[\chi^2_{Nn}(0.025),\chi^2_{Nn}(0.975)]/N$，可用 Wilson–Hilferty 近似。
8. 实验：$Q$ 小 10 倍 → ANEES 10.63（理论 2）；匹配 → 1.954；$Q$ 大 10 倍 → 1.153。
9. 新息一阶自相关符号指示 $Q$ 调整方向：正 ⇒ $Q$ 偏小，负 ⇒ 偏大。
10. 调参清单：无噪声验证 → $R$ → $Q$ → 仿真 MC → 实机 NIS 与自相关 → 检查不可观方向 → 压力测试 → 记录。

## 15. 自测题

1. $\mathcal O$ 的秩为 1 的二维系统，不可观子空间维数？（1。）
2. 20 次运行，$n=3$，ANEES 的 95% 区间用的自由度？（$Nn=60$，区间为 $\chi^2_{60}$ 的分位数除以 20。）
3. 新息平均 NIS 为 4，测量维数 $m=2$（时间平均），说明什么？（高于理论 2，滤波器偏自信或模型/噪声有问题。）
4. 为什么只用速度类测量，位置协方差必然增长？（位置不可观，速度估计误差被积分，加上过程噪声。）
5. 机器人与路标做相对测量，应当锚定几个自由度？（3 个：全局平移 2 + 旋转 1。）
6. 新息一阶自相关为 $+0.2$，应调大还是调小 $Q$？（调大 $Q$，滤波器反应过慢。）

## 16. 相关链接

- 上一章：[误差状态卡尔曼 ESKF 与流形上的滤波](./09_误差状态卡尔曼ESKF与流形上的滤波.md)；模块首章：[传感器：IMU、编码器、激光雷达](./01_传感器_IMU_编码器_激光雷达.md)
- 相关基础：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[状态空间：可控可观、稳定性](../05_控制理论/03_状态空间_可控可观_稳定性.md)、[SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)
- 项目：[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 模块说明：[06_状态估计 README](./README.md)
