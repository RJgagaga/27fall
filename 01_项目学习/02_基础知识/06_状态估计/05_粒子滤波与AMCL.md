# 粒子滤波与 AMCL

> **一句话定位**：粒子滤波用一堆带权重的"候选位姿"去表示机器人可能在哪，能表达多峰、非高斯的信念；AMCL（Adaptive Monte Carlo Localization）是它在"已知地图 + 2D 激光"上的标准工程实现。这是 P5 里 "EKF/AMCL" 的 AMCL 那一半。

## 本章学完你能……

- 解释为什么全局定位需要粒子滤波而不是 EKF：多峰、非高斯、初始位置未知。
- 从重要性采样推导粒子滤波的三步（按运动模型采样、按观测加权、重采样），并解释"权重 = 似然"为什么成立。
- 手算一次完整的一维粒子滤波：权重、归一化、有效粒子数、系统重采样。
- 说出里程计运动模型（rot1 / trans / rot2）和噪声参数 $\alpha_1\ldots\alpha_4$ 的含义，并手算噪声标准差。
- 解释似然域观测模型（likelihood field）：为什么比光束模型好用，怎样用距离变换，手算几条光束的权重。
- 理解 AMCL 的自适应：KLD 采样（粒子数怎么随不确定度变化）、增强 MCL 的随机注入（kidnapped robot 恢复），以及 `map→odom` 是怎么算出来的。
- 列出 AMCL 的典型失败模式与排查方法，用英文回答面试问题。

## 前置知识

- 概率基础与贝叶斯公式：[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)
- 贝叶斯滤波两步：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md) 第 1 节
- 激光数据与坐标：[传感器章](./01_传感器_IMU_编码器_激光雷达.md)、里程计：[里程计与传感器融合](./04_里程计与传感器融合.md)

## 目录

1. 为什么需要粒子滤波
2. 粒子滤波的基本流程
3. 重要性采样：权重为什么等于似然
4. 重采样与退化
5. 手算：一维走廊的完整一轮
6. 运动模型
7. 观测模型：光束模型与似然域
8. AMCL 的工程结构
9. KLD 自适应粒子数
10. 初始化、全局定位与 kidnapped 恢复
11. `map→odom` 是怎么得到的
12. 失败模式与排查
13. 在本项目里怎么用
14. 面试怎么问、怎么答
15. 本章要点速记、自测题、相关链接

---

## 1. 为什么需要粒子滤波

### 1.1 它要解决什么问题

假设机器人被放进一座已知地图的大楼，不知道自己在哪个房间。走廊长得相似，所以"我可能在 A 走廊，也可能在 B 走廊"。信念（belief）有**多个峰**。

EKF 的信念是**一个高斯**：一个均值 + 一个协方差椭圆。它没法同时表示 A 和 B 两个峰。强行用一个高斯，均值会落在 A 和 B 中间的墙里，既不对应 A 也不对应 B。

粒子滤波（Particle Filter），在定位里也叫蒙特卡洛定位（Monte Carlo Localization, MCL）：

- 用 $N$ 个**样本**（粒子）$\{x^{(i)}\}$ 表示信念，每个粒子是一个完整的位姿 $(x,y,\theta)$；
- 每个粒子带一个**权重** $w^{(i)}$，表示它有多可信；
- 粒子密集的地方概率高，可以同时有多个簇。

### 1.2 与 EKF 的对比

| | EKF | 粒子滤波 / AMCL |
|---|---|---|
| 信念表示 | 一个高斯 | 带权样本集 |
| 多峰 | 不能 | 能 |
| 全局定位 | 需要好的初值 | 可以从均匀分布开始 |
| 非线性 | 局部线性化 | 无需线性化 |
| 计算量 | 小（矩阵运算） | 大（粒子数 × 光束数） |
| 输出 | 平滑、连续 | 可能有抖动（采样噪声） |
| 典型用途 | 局部、连续融合 | 全局、基于地图的定位 |

所以工程上两者**互补**：EKF 给连续的局部状态，AMCL 给基于地图的全局位姿。

---

## 2. 粒子滤波的基本流程

符号：

| 符号 | 含义 |
|---|---|
| $x_k^{(i)}$ | 第 $i$ 个粒子在 $k$ 时刻的位姿 |
| $w_k^{(i)}$ | 第 $i$ 个粒子的权重（归一化后 $\sum_iw^{(i)}=1$） |
| $u_k$ | 控制 / 里程计增量 |
| $z_k$ | 激光测量 |
| $m$ | 地图 |
| $N$ | 粒子数 |

每个周期：

1. **预测（采样）**：对每个粒子，按运动模型采样新位姿 $x_k^{(i)}\sim p(x_k\mid x_{k-1}^{(i)},u_k)$。噪声让粒子散开。
2. **加权（更新）**：计算 $w_k^{(i)}\propto w_{k-1}^{(i)}\,p(z_k\mid x_k^{(i)},m)$。与激光和地图吻合的粒子权重高。
3. **归一化**：$w^{(i)}\leftarrow w^{(i)}/\sum_jw^{(j)}$。
4. **重采样**：按权重复制高权重粒子、丢弃低权重粒子，权重重置为 $1/N$。
5. **估计**：输出加权均值（或最高权重簇的均值）和协方差。

```text
 粒子云(上一刻)        运动模型+噪声        激光 vs 地图打分       重采样
 · · ·  ·             ·  · ·  ·            · · ●  ·              ● ● ● ·
  ·  · ·      ───▶     ·  ·  · ·   ───▶    ·  ● ● ·    ───▶      ●  ● ·
 · ·  ·                · · ·  ·             · · ●  ·              ● ● ·
                       (散开)               (●=高权重)           (向高权重集中)
```

---

## 3. 重要性采样：权重为什么等于似然

### 3.1 要解决什么问题

我们想要的是**目标分布**（后验）$p(x_k\mid z_{1:k},u_{1:k})$ 的样本，但无法直接从它采样。

**重要性采样**（importance sampling）：从一个容易采样的**提议分布** $q(x)$ 采样，再用权重 $w=p(x)/q(x)$ 修正，使加权样本等价于来自 $p$ 的样本：

$$
E_p[g(x)]=\int g(x)\,p(x)\,dx=\int g(x)\,\frac{p(x)}{q(x)}\,q(x)\,dx\approx\frac1N\sum_i w^{(i)}g(x^{(i)}),\quad x^{(i)}\sim q,\ w^{(i)}=\frac{p(x^{(i)})}{q(x^{(i)})}
$$

### 3.2 在定位里的应用

贝叶斯滤波的更新：

$$
p(x_k\mid z_{1:k})=\eta\ p(z_k\mid x_k)\ \underbrace{\int p(x_k\mid x_{k-1},u_k)\,p(x_{k-1}\mid z_{1:k-1})\,dx_{k-1}}_{\text{预测分布}}
$$

取**提议分布 = 预测分布**（也就是"按运动模型传播上一时刻的粒子"，容易采样）。则

$$
w^{(i)}=\frac{\text{目标}}{\text{提议}}=\frac{\eta\ p(z_k\mid x_k^{(i)})\ \text{预测}(x_k^{(i)})}{\text{预测}(x_k^{(i)})}=\eta\ p(z_k\mid x_k^{(i)})
$$

预测项在分子分母约掉，**权重就是观测似然**（归一化常数 $\eta$ 事后统一除掉）。这就是粒子滤波"按运动模型采样、按观测加权"的数学来由。

---

## 4. 重采样与退化

### 4.1 为什么要重采样

只加权不重采样，几轮后少数粒子权重趋近 1，其余趋近 0，大量计算浪费在无用的粒子上。这叫**粒子退化**（degeneracy）。

**有效粒子数**（effective sample size）：

$$
N_{eff}=\frac1{\sum_i\big(w^{(i)}\big)^2}
$$

- 所有权重相等 $=1/N$：$N_{eff}=N$；
- 一个粒子权重为 1：$N_{eff}=1$。

当 $N_{eff}<N/2$（常用阈值）时重采样。

### 4.2 重采样方法

**多项式重采样**：每次独立地按权重抽一个粒子，抽 $N$ 次。方差较大。

**系统重采样**（systematic / low-variance resampling）：只用一个随机数 $u_0\sim U[0,1/N)$，取 $N$ 个等间隔的点 $u_j=u_0+j/N$，看每个点落在累积权重的哪一段。方差小、$O(N)$ 复杂度，最常用。

### 4.3 重采样的副作用

- **多样性丢失**：同一个粒子被复制很多份，后续靠运动噪声重新散开。
- **粒子枯竭**（particle depletion）：如果运动模型噪声太小，复制出来的粒子不会散开，一旦估计偏了就回不来。
- 所以**不要每次都重采样**：AMCL 有 `resample_interval` 参数，每隔几次更新才重采样一次。

---

## 5. 手算：一维走廊的完整一轮

### 5.1 设定（教学数值）

机器人在一条一维走廊上，位置 $x$ 单位 m。机器人有一个测距传感器，测到离**走廊左端墙**的距离。有 5 个粒子：位置分别为 $1,2,3,4,5$ m，初始权重各 $0.2$。传感器读数 $z=2.4$ m，测量噪声标准差 $\sigma=0.5$ m。这里观测模型直接是 $p(z\mid x)\propto\exp\!\big(-\tfrac12(\tfrac{z-x}{\sigma})^2\big)$。

### 5.2 加权

$$
\tilde w^{(i)}=\exp\Big(-\tfrac12\big(\tfrac{2.4-x^{(i)}}{0.5}\big)^2\Big)
$$

| 粒子 | $x$ | $(z-x)/\sigma$ | $\tilde w$ |
|---|---|---|---|
| 1 | 1 | $-2.8$ | $e^{-3.92}=0.0198$ |
| 2 | 2 | $-0.8$ | $e^{-0.32}=0.7261$ |
| 3 | 3 | $+1.2$ | $e^{-0.72}=0.4868$ |
| 4 | 4 | $+3.2$ | $e^{-5.12}=0.0060$ |
| 5 | 5 | $+5.2$ | $e^{-13.52}\approx0.0000$ |

求和 $=0.0198+0.7261+0.4868+0.0060+0.0000=1.2387$。归一化：

| 粒子 | $w$ |
|---|---|
| 1 | 0.0160 |
| 2 | 0.5862 |
| 3 | 0.3930 |
| 4 | 0.0048 |
| 5 | 0.0000 |

### 5.3 有效粒子数

$$
N_{eff}=\frac1{0.0160^2+0.5862^2+0.3930^2+0.0048^2+0}=\frac1{0.000256+0.34363+0.15445+0.000023}=\frac1{0.49836}=2.007
$$

$N_{eff}=2.0<N/2=2.5$，需要重采样。

### 5.4 系统重采样

累积权重 $c$：粒子 1：0.0160；粒子 2：0.6022；粒子 3：0.9952；粒子 4：1.0000；粒子 5：1.0000。

取 $N=5$，$u_0=0.12$（$\in[0,0.2)$），采样点 $u_j=0.12+j\times0.2$：$0.12,\ 0.32,\ 0.52,\ 0.72,\ 0.92$。

| 采样点 | 落在哪一段 | 选中粒子 |
|---|---|---|
| 0.12 | 0.0160 ~ 0.6022 | 2（x=2） |
| 0.32 | 同上 | 2 |
| 0.52 | 同上 | 2 |
| 0.72 | 0.6022 ~ 0.9952 | 3（x=3） |
| 0.92 | 同上 | 3 |

重采样后：粒子位置 $\{2,2,2,3,3\}$，权重各 $0.2$。

### 5.5 下一轮：运动与散开

机器人向右移动 $u=1$ m，运动噪声使每个粒子加上 $N(0,0.2^2)$ 的随机量，例如：$2.17,\ 2.91,\ 3.06,\ 4.12,\ 3.88$（抽样示例）。三个原来在 2 的粒子被噪声拉开，不再重合；这就是运动噪声对抗多样性丢失的作用。

> **提示**：重采样后粒子云集中在 2~3 m 附近，但真值可能在 2.4。粒子数 5 太少只是为了手算；真实 AMCL 用数百到数千粒子。

---

## 6. 运动模型

### 6.1 里程计运动模型（Thrun 的 sample_motion_model_odometry）

AMCL 默认的"差速模型"把两帧里程计位姿之间的相对运动分解为三步：先转 $\delta_{rot1}$，再直线走 $\delta_{trans}$，最后转 $\delta_{rot2}$。

设里程计在 $t-1$ 时位姿 $(\bar x,\bar y,\bar\theta)$，在 $t$ 时位姿 $(\bar x',\bar y',\bar\theta')$：

$$
\delta_{trans}=\sqrt{(\bar x'-\bar x)^2+(\bar y'-\bar y)^2}
$$
$$
\delta_{rot1}=\operatorname{atan2}(\bar y'-\bar y,\ \bar x'-\bar x)-\bar\theta,\qquad \delta_{rot2}=\bar\theta'-\bar\theta-\delta_{rot1}
$$

给每个粒子加噪声（$\alpha_1\ldots\alpha_4$ 为噪声参数）：

$$
\hat\delta_{rot1}=\delta_{rot1}-\epsilon\big(\alpha_1\delta_{rot1}^2+\alpha_2\delta_{trans}^2\big)
$$
$$
\hat\delta_{trans}=\delta_{trans}-\epsilon\big(\alpha_3\delta_{trans}^2+\alpha_4(\delta_{rot1}^2+\delta_{rot2}^2)\big)
$$
$$
\hat\delta_{rot2}=\delta_{rot2}-\epsilon\big(\alpha_1\delta_{rot2}^2+\alpha_2\delta_{trans}^2\big)
$$

这里 $\epsilon(\sigma^2)$ 表示一个方差为 $\sigma^2$ 的零均值高斯随机数。然后粒子位姿更新：

$$
x'=x+\hat\delta_{trans}\cos(\theta+\hat\delta_{rot1}),\quad y'=y+\hat\delta_{trans}\sin(\theta+\hat\delta_{rot1}),\quad \theta'=\theta+\hat\delta_{rot1}+\hat\delta_{rot2}
$$

### 6.2 $\alpha$ 的含义

| 参数 | 含义 |
|---|---|
| $\alpha_1$ | 旋转引起的旋转误差 |
| $\alpha_2$ | 平移引起的旋转误差 |
| $\alpha_3$ | 平移引起的平移误差 |
| $\alpha_4$ | 旋转引起的平移误差 |

### 6.3 手算噪声标准差

**示例**：$\alpha_1=\alpha_2=\alpha_3=\alpha_4=0.1$，$\delta_{rot1}=0.1$ rad，$\delta_{trans}=1.0$ m，$\delta_{rot2}=-0.1$ rad。

- $\sigma_{rot1}=\sqrt{0.1\times0.1^2+0.1\times1.0^2}=\sqrt{0.001+0.1}=0.318$ rad $=18.2°$
- $\sigma_{trans}=\sqrt{0.1\times1.0^2+0.1\times(0.1^2+0.1^2)}=\sqrt{0.1+0.002}=0.319$ m
- $\sigma_{rot2}=\sqrt{0.1\times0.1^2+0.1\times1.0^2}=0.318$ rad

走 1 m，粒子的航向和距离不确定度都在 0.3 量级。这是一个非常保守（大）的设置。

### 6.4 车式机器人的特殊性

AMCL 内置的运动模型一般只有**差速**（diff）和**全向**（omni）两类。车式（Ackermann）底盘不能原地转、有转弯半径限制，直接套差速模型是个近似。常见处理：

- 仍使用差速模型，但适当**加大噪声** $\alpha$，让粒子散得够开；
- 或在里程计输入前保证 `odom` 的 twist / 位姿来自车式运动学（见 [里程计章](./04_里程计与传感器融合.md)）。

我们的做法是沿用 AMCL 的差速型运动模型，但把噪声 $\alpha_1..\alpha_4$ 适当放大（量级取 0.1–0.3），让粒子散得足够开，同时保证输入的 `odom` 来自车式运动学。

---

## 7. 观测模型：光束模型与似然域

### 7.1 要解决什么问题

给定一个假设位姿 $x$、地图 $m$ 和一帧激光 $z=\{z^1,\dots,z^K\}$，计算 $p(z\mid x,m)$：激光与"在这个位姿下，地图预期看到的"吻合得有多好。

### 7.2 光束模型（beam model）

对每条光束，沿射线在地图里做**光线投射**（ray casting）得到期望距离 $z^{*}$，再用混合分布描述实测 $z$ 与 $z^*$ 的关系：命中（高斯）、被动态物体挡住（指数）、最大量程、随机噪声。物理上完整，但：

- 光线投射**计算昂贵**（每个粒子、每条光束都要投射）；
- 对位姿**不平滑**：位姿稍微变一点，光线打到的墙格子会跳变，似然曲面粗糙，粒子数少时效果差。

### 7.3 似然域模型（likelihood field）

换一个思路：不问"这条光束应该打到哪"，而问"**这条光束的终点离地图上最近的障碍物有多远**"。

步骤：

1. 预处理地图：对每个栅格计算到最近障碍物的距离 $d(x,y)$——**距离变换**（distance transform），整张图算一次。
2. 对每条光束，用位姿 $(x,y,\theta)$ 和传感器外参，把光束终点投影到地图坐标 $(x_z,y_z)$。
3. 查表得 $d=d(x_z,y_z)$，光束概率：

$$
p(z^k\mid x,m)=z_{hit}\,\exp\!\Big(-\frac{d^2}{2\sigma_{hit}^2}\Big)+\frac{z_{rand}}{z_{max}}
$$

   第一项：终点离障碍物越近越可能；第二项：均匀随机噪声基线（保证不为零）。

4. 一帧激光的似然：各光束（假设独立）连乘：$p(z\mid x,m)=\prod_kp(z^k\mid x,m)$。

**优点**：只需查表，不需要光线投射，快；曲面平滑；对地图中没有的动态障碍物（终点离墙很远）只会让那几条光束概率落到基线 $z_{rand}/z_{max}$，不至于让整帧归零。

> 在 ROS 的 amcl 里，`laser_model_type` 可选 `beam`、`likelihood_field`（默认）和 `likelihood_field_prob`。参数 `laser_z_hit`、`laser_z_rand`、`laser_sigma_hit`、`laser_max_range`、`max_beams`（只用均匀抽取的一部分光束）等。参数名与默认值以所用版本文档为准（凭记忆，未逐项核对）。

### 7.4 手算

**设定**（教学数值）：$\sigma_{hit}=0.2$ m，$z_{hit}=0.95$，$z_{rand}=0.05$，$z_{max}=10$ m，所以基线 $z_{rand}/z_{max}=0.005$。每个粒子取 3 条光束，查表得到终点到最近障碍物的距离 $d$：

| 粒子 | $d$ 值（m） | 各光束概率 $p$ | 乘积 |
|---|---|---|---|
| A（位置接近真值） | 0.05, 0.10, 0.00 | 0.9258, 0.8434, 0.9550 | 0.746 |
| B（位置偏离） | 0.60, 0.80, 0.50 | 0.0156, 0.0053, 0.0467 | $3.9\times10^{-6}$ |
| C（有一条被人挡了） | 0.10, 0.10, 0.80 | 0.8434, 0.8434, 0.0053 | 0.00378 |

计算示例（粒子 A 第一束）：$0.95\exp(-0.5\times(0.05/0.2)^2)+0.005=0.95\times0.9692+0.005=0.9258$。

观察：

1. A 比 B 高 5 个数量级。连乘使权重**对错误极度敏感**。
2. 粒子 C 有一条光束被动态障碍物遮挡（d=0.8），乘积降到 0.00378，仍比 B 高三个数量级；基线 0.005 保证了一条坏光束不会把权重压成零。
3. 连乘在光束数多、光束相关（相邻光束一起错）时会**过度自信**。amcl 实现里有一个常见处理：对每条光束概率取立方再累加（$p\leftarrow1+\sum_kp_k^3$）来压缩动态范围，避免权重过度极端、粒子退化。这是实现细节，【未核对源码，面试时说"实现里会对权重做平滑/压缩"即可】。

粒子 A、B、C 取 $p=1+\sum p_k^3$ 的值分别为 $3.26,\ 1.0001,\ 2.20$，相对差别小得多（A:C:B≈3.26:2.20:1.00），更平滑。

---

## 8. AMCL 的工程结构

AMCL = Adaptive Monte Carlo Localization，是 MCL 加上两项**自适应**：

1. **KLD 采样**：根据不确定度自动调整粒子数（第 9 节）；
2. **增强 MCL（augmented MCL）**：根据长期 / 短期平均观测似然的比值，向粒子集**注入随机粒子**，从 kidnapped 或定位丢失中恢复（第 10 节）。

工作流程（ROS amcl 节点）：

```text
 /map ──▶ 载入地图、算距离场
 /initialpose 或 全局初始化 ──▶ 初始化粒子云
 /scan + TF(odom→base_link) ──▶
     1. 机器人移动超过 update_min_d / update_min_a 才触发一次更新（避免静止时反复重采样）
     2. 按运动模型更新粒子
     3. 按似然域模型加权
     4. 每 resample_interval 次更新重采样一次（含 KLD 与随机注入）
     5. 取最大权重簇（或加权平均）为位姿估计
 ──▶ 发布 /amcl_pose（带协方差）、/particlecloud、TF: map→odom
```

> 关键点：AMCL **不发布 `odom→base_link`**；它读取这条变换作为运动输入，发布的是 `map→odom`（第 11 节）。

---

## 9. KLD 自适应粒子数

### 9.1 它要解决什么问题

位姿不确定度大时（全局定位）需要很多粒子；不确定度小时（已锁定）几十个粒子就够，再多是浪费算力。KLD 采样（Fox, 2003）根据粒子占据的**离散格子数 $k$** 动态决定粒子数 $N$。

### 9.2 思想

把状态空间划成固定大小的格子（bin，位置、航向各一个尺度）。粒子集占据的非空格子数 $k$ 越多，说明分布越分散，需要的粒子越多。要求：以概率 $1-\delta$，样本近似与真实分布之间的 KL 散度不超过 $\varepsilon$。用卡方分布近似（Wilson–Hilferty 变换）得到所需粒子数：

$$
N=\frac{k-1}{2\varepsilon}\left\{1-\frac{2}{9(k-1)}+\sqrt{\frac{2}{9(k-1)}}\ z_{1-\delta}\right\}^3
$$

其中 $z_{1-\delta}$ 是标准正态的上分位数。

### 9.3 手算

**示例**：$\varepsilon=0.05$，$\delta=0.01$（$z_{0.99}=2.326$）。

- $k=2$（锁定得很好，只占 2 个格子）：$N=\frac1{0.1}\{1-\frac29+\sqrt{\frac29}\times2.326\}^3=10\times\{1-0.2222+0.4714\times2.326\}^3=10\times\{1.8742\}^3=10\times6.583=65.8\approx66$。
- $k=10$：$N=\frac9{0.1}\{1-\frac2{81}+\sqrt{\frac2{81}}\times2.326\}^3=90\times\{1-0.02469+0.1571\times2.326\}^3=90\times\{1.3408\}^3=90\times2.4104\approx217$。
- $k=50$：约 $749$。

粒子数大致随 $k$ 线性增长。AMCL 再用 `min_particles`、`max_particles` 做上下限。

> 参数：`kld_err` 对应 $\varepsilon$，`kld_z` 对应 $1-\delta$ 的分位。格子尺寸典型约 0.5 m × 0.5 m × 10°（凭记忆，以源码为准）。

---

## 10. 初始化、全局定位与 kidnapped 恢复

### 10.1 初始化的三种方式

1. **给定初始位姿**（RViz 的 "2D Pose Estimate"，话题 `/initialpose`）：粒子在该位姿附近按给定协方差散开。最快、最稳，也是实机最常用。
2. **全局初始化**（`global_localization` 服务）：粒子在地图所有自由空间均匀撒开，靠机器人运动 + 观测逐渐收敛。需要环境有足够的区分度。
3. **用上一次关机时保存的位姿**（需要自己实现存取）。

### 10.2 全局定位如何收敛

均匀撒 $N$ 个粒子，每次观测都会使与地图不吻合的粒子权重降低，重采样后集中到相似位置的几个簇；机器人移动并观测到不同区域，错误簇被淘汰。走廊对称时，可能需要移动更远才能区分。

### 10.3 Kidnapped robot 与增强 MCL

**问题**：机器人被"绑架"（被人搬到别处）或定位错误时，所有粒子都离真值很远，重采样只会复制错误粒子，不会自己恢复。

**增强 MCL 的做法**：维护两个观测似然的滑动平均：

$$
w_{slow}\leftarrow w_{slow}+\alpha_{slow}(w_{avg}-w_{slow}),\qquad w_{fast}\leftarrow w_{fast}+\alpha_{fast}(w_{avg}-w_{fast}),\quad \alpha_{fast}\gg\alpha_{slow}
$$

其中 $w_{avg}$ 是当前所有粒子权重的平均。重采样时以概率

$$
p_{rand}=\max\Big(0,\ 1-\frac{w_{fast}}{w_{slow}}\Big)
$$

用随机位姿替换一个粒子。直觉：如果最近的观测似然（快平均）突然低于长期水平（慢平均），说明"粒子云不再解释观测"，就增加随机粒子去探索。

**手算**：初始 $w_{slow}=w_{fast}=0.5$，$\alpha_{slow}=0.1$，$\alpha_{fast}=0.5$。机器人被搬走后连续三轮平均似然 $w_{avg}=0.1$：

| 轮 | $w_{slow}$ | $w_{fast}$ | $p_{rand}=1-w_{fast}/w_{slow}$ |
|---|---|---|---|
| 1 | $0.5+0.1(0.1-0.5)=0.460$ | $0.5+0.5(0.1-0.5)=0.300$ | 0.348 |
| 2 | 0.424 | 0.200 | 0.528 |
| 3 | 0.392 | 0.150 | 0.617 |

三轮后超过一半的粒子被随机替换，为恢复探索创造了机会。

AMCL 参数：`recovery_alpha_slow`、`recovery_alpha_fast`，默认都是 0，即**默认关闭**随机注入；需要恢复能力时要显式设置（参数默认值凭记忆，以文档为准）。

---

## 11. `map→odom` 是怎么得到的

AMCL 的粒子估计的是车在 `map` 系下的位姿 $T_{map\,base}$。但 TF 树里规定 `base_link` 的父节点是 `odom`，`odom` 的父节点是 `map`（REP-105，见[导航系统架构](../08_规划与导航/05_导航系统架构_Nav2与move_base对比.md)）。所以 AMCL 不发布 `map→base_link`，而是发布：

$$
T_{map\,odom}=T_{map\,base}\cdot\big(T_{odom\,base}\big)^{-1}
$$

**手算（示例）**：AMCL 估计 $T_{map\,base}=(10,\ 5,\ 90°)$；里程计 $T_{odom\,base}=(2,\ 1,\ 30°)$。

- 旋转：$\theta_{mo}=90°-30°=60°$；
- 平移：$t_{mo}=p_{mb}-R(60°)\,p_{ob}$，$R(60°)\binom21=\binom{2\cos60°-1\sin60°}{2\sin60°+1\cos60°}=\binom{0.134}{2.232}$；
- $t_{mo}=(10-0.134,\ 5-2.232)=(9.866,\ 2.768)$。

所以 `map→odom` $=(9.866,\ 2.768,\ 60°)$。验证：把 `odom` 系中的 $(2,1,30°)$ 变到 `map` 系：$R(60°)(2,1)+t=(0.134,2.232)+(9.866,2.768)=(10,5)$ ✓，角度 $30°+60°=90°$ ✓。

这样安排的意义：

- `odom→base_link`（来自 EKF / 里程计）**始终连续**；
- AMCL 每次校正，只改变 `map→odom`——当 AMCL 修正累积漂移时，`map→odom` 会发生**阶跃**，但车自己的 `odom→base_link` 不跳。控制器如果在 `odom` 系下工作，就看不到这个跳变（见 [时间同步与位姿跳变](./06_时间同步_坐标系一致性与数据有效性.md)）。
- AMCL 发布 TF 时会带一个**未来时间戳**（`transform_tolerance`），使下游查询不会因为 AMCL 更新较慢（几 Hz）而外推失败。

---

## 12. 失败模式与排查

| 现象 | 常见原因 | 排查 / 处理 |
|---|---|---|
| 粒子云始终散得很开不收敛 | 初始位姿太差；环境对称；激光外参或 `base_link` 到 `laser` 变换错误 | 在 RViz 给出准确初始位姿；检查激光点与地图墙是否对齐 |
| 粒子云收敛到错误位置并锁死 | 粒子枯竭；运动噪声过小；错误簇被复制 | 加大 $\alpha$；开 `recovery_alpha_*`；给更好的初值 |
| 长走廊中沿走廊方向滑动 | 几何退化：激光只约束横向 | 靠里程计 + 其他特征；必要时增加反光贴 / 特征 |
| 动态障碍多（人群）时偏离 | 光束被遮挡，似然被拉低 | 增大 $z_{rand}$；`max_beams` 取少量；地图保持更新 |
| 位姿频繁小幅跳动 | 粒子数偏少；重采样过频；观测过度自信 | 增 `min_particles`；加大 `resample_interval`；压缩权重 |
| `map→odom` 突然大跳变 | 里程计漂移被一次性校正；或错误簇接管 | 监控跳变量，限幅 / 门控；见时间同步章 |
| 转弯时定位变差 | 里程计角度噪声参数偏小；时间戳不同步 | 检查 TF 时间戳与激光时间戳；加大旋转噪声 |
| 地图与现实不符（家具挪动） | 地图过期 | 更新地图 |
| 日志提示 "Failed to transform" / extrapolation | 激光时间戳超出 TF 缓冲 | 检查时钟同步，调大 `transform_tolerance` |
| CPU 占用高 | 粒子数大、光束数多 | `max_particles` 下调；`max_beams` 下调；降低激光频率 |

排查顺序建议：**先看 TF 与时间，再看地图与外参，最后调参**。多数"AMCL 不准"其实是 TF / 时间 / 外参问题，而不是算法本身。

---

## 13. 在本项目里怎么用

CV 的 P5 条目：EKF/AMCL 融合 IMU、轮式编码器和 2D LiDAR 做基于地图的定位，并输出连续的车辆状态。结合本章：

- **AMCL 提供**：LiDAR + 已知地图 → `map` 系全局位姿（经 `map→odom`）；
- **EKF 提供**：IMU + 轮速 → 连续的 `odom→base_link`，也是 AMCL 的运动输入；
- **车式底盘**：AMCL 内置运动模型是差速 / 全向，所以需要注意噪声参数设置（第 6.4 节）；
- **位姿跳变**：AMCL 校正造成 `map→odom` 阶跃；时间不同步、坐标系错误、里程计方向错误会让 AMCL 的粒子云不稳定，表现为跳变。

详见 [卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[里程计与传感器融合](./04_里程计与传感器融合.md)、[P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **本项目通常的 AMCL 设置（量级，调参时按实机微调）**：粒子数范围取 `min_particles` 500、`max_particles` 2000–5000 的量级；激光模型选似然域（`likelihood_field`），计算量小且对小误差更平滑；`update_min_d` 取 0.1–0.2 m、`update_min_a` 取 0.2 rad 左右，避免静止时反复更新；初始化采用手动给初始位姿（RViz 的 2D Pose Estimate），必要时再用全局定位。典型失败案例是倒车符号错误或激光与 `base_link` 的外参不准，导致粒子云被拉扯、位姿跳变，修复是改正里程计方向并核对 TF；整体上我们使用并调整了现成的 AMCL 配置。

---

## 14. 面试怎么问、怎么答

**Q1. Why use AMCL instead of just an EKF for map-based localisation?**

> A: An EKF represents the belief as a single Gaussian, so it needs a good initial guess and cannot hold several hypotheses. AMCL uses particles, so it can represent multi-modal beliefs and recover from a bad start. In my navigation stack the two complement each other: the EKF gives a smooth local odometry frame, and AMCL uses the LiDAR and the map to correct the global drift.

中文要点：EKF 单高斯，需好初值；AMCL 粒子，多峰、可全局定位；二者互补。

**Q2. Explain the particle filter steps.**

> A: I sample each particle forward with the motion model plus noise, then weight it by how well the laser scan matches the map at that pose, normalise the weights, and resample so that high-weight particles are copied and low-weight ones are dropped. The weight equals the measurement likelihood because the proposal distribution is the motion prediction.

追问：重采样的副作用？（答：多样性丢失、粒子枯竭；用有效粒子数 $N_{eff}$ 决定何时重采样，运动噪声保证散开。）

**Q3. What is KLD sampling?**

> A: It adapts the number of particles to the uncertainty. I count how many bins of the state space contain particles, and the formula gives the number of samples needed to keep the KL divergence to the true posterior under a threshold with a given probability. So when the pose is well known I use few particles, and when it is uncertain, like at the start of global localisation, I use many.

**Q4. What makes AMCL fail, and what do you check first?**

> A: Typical causes are a wrong initial pose, symmetric or featureless environments like long corridors, dynamic obstacles, a stale map, and wrong sensor extrinsics. Before touching parameters I check TF and timestamps, then whether the laser points line up with the map walls in RViz. Then I look at motion noise and particle count.

**Q5. How is the map to odom transform computed?**

> A: AMCL estimates the pose of the base in the map. Since the base's parent in the TF tree is odom, it publishes map-to-odom as the map-to-base pose multiplied by the inverse of the odom-to-base transform. That keeps odom to base continuous, and all corrections show up in map to odom.

## 15. 本章要点速记

1. 粒子 = 完整位姿假设 + 权重；能表示多峰、非高斯信念。
2. 流程：按运动模型采样 → 按观测加权 → 归一化 → 重采样。
3. 提议分布取运动预测，则权重 = 观测似然。
4. $N_{eff}=1/\sum w^2$；小于 $N/2$ 重采样；系统重采样方差小。
5. 重采样副作用：多样性丢失；运动噪声和 `resample_interval` 是对策。
6. 里程计运动模型：rot1 / trans / rot2 + $\alpha_{1..4}$；车式底盘用差速模型是近似，需调大噪声。
7. 似然域：距离变换 + 查表，$p=z_{hit}e^{-d^2/2\sigma^2}+z_{rand}/z_{max}$，快且平滑。
8. KLD：粒子数 $N\propto k$（非空格子数），锁定后粒子少。
9. 增强 MCL：$p_{rand}=\max(0,1-w_{fast}/w_{slow})$ 注入随机粒子；默认关闭。
10. AMCL 发布 `map→odom`，不发布 `odom→base_link`；校正的阶跃体现在 `map→odom`。

## 自测题

1. 4 个粒子权重 $(0.5,0.3,0.1,0.1)$，求 $N_{eff}$，是否重采样（阈值 $N/2$）？（答：$1/(0.25+0.09+0.01+0.01)=2.78>2$，不重采样。）
2. 系统重采样，$N=4$，权重 $(0.1,0.4,0.4,0.1)$，$u_0=0.05$，选中哪几个？（答：累积 0.1,0.5,0.9,1.0；采样点 0.05,0.30,0.55,0.80 → 粒子 1,2,3,3。）
3. 似然域：$d=0.4$，$\sigma=0.2$，$z_{hit}=0.9$，$z_{rand}/z_{max}=0.01$，求 $p$。（答：$0.9e^{-2}+0.01=0.9×0.1353+0.01=0.1318$。）
4. 为什么运动噪声设为 0 的 AMCL 容易在偏了之后无法恢复？（要点：重采样后粒子重叠，没有散开去覆盖真值附近，发生粒子枯竭。）
5. $T_{map\,base}=(5,0,0°)$，$T_{odom\,base}=(1,0,0°)$，求 `map→odom`。（答：旋转 0，平移 $(5-1,0)=(4,0)$。）
6. 为什么似然域比光束模型对位姿更平滑？（要点：光束模型光线终点随位姿微变会在格子间跳变；似然域用到最近障碍物的连续距离。）
7. 全局定位时为什么需要很多粒子，锁定后却可以减少？（要点：不确定度大，占据格子数 $k$ 大；KLD 公式中 $N$ 随 $k$ 增长。）

## 相关链接

- 上一章：[里程计与传感器融合](./04_里程计与传感器融合.md)
- 下一章：[时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)
- 相关：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[导航系统架构：Nav2 与 move_base 对比](../08_规划与导航/05_导航系统架构_Nav2与move_base对比.md)
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 八股：[状态估计与 SLAM 八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
