# 05 AMCL：粒子滤波的地图定位

> **一句话定位**：AMCL（Adaptive Monte Carlo Localization）用一群"假设位姿"（粒子）在已知地图上猜车在哪，用激光扫描给每个猜测打分，再按分数"优胜劣汰"。它不累积漂移，是 P5 里"全局位置"的来源；代价是输出会有**离散的修正**，也就是位姿跳变的主要源头之一。

**本章学完你能：**

- 说清贝叶斯滤波的两步（运动预测 + 观测更新）以及粒子滤波如何用样本近似它；
- 解释 AMCL 的运动模型（里程计模型）和观测模型（似然域，likelihood field），并手算两个粒子的权重；
- 手算低方差重采样、KLD 采样所需粒子数、有效粒子数 $N_{eff}$；
- 解释 kidnapped robot、全局定位、恢复机制（augmented MCL）和初始位姿的作用；
- 说出 AMCL 关键参数及其含义，并针对车式小车（无原地转向、会倒车）解释应注意什么；
- 分析 AMCL 失效模式：长走廊、对称环境、玻璃、动态障碍、粒子耗尽。

**前置知识：**

- 粒子滤波与 AMCL（基础）：[../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md](../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md)
- 概率统计：[../../02_基础知识/01_数学/03_概率统计与高斯分布.md](../../02_基础知识/01_数学/03_概率统计与高斯分布.md)
- 本项目：[04 EKF](./04_EKF融合定位.md)、[03 传感器与数据质量](./03_传感器与数据质量_时间坐标与运动方向.md)、[07 代价地图与车体外形](./07_代价地图与车体外形.md)

**目录**

1. 为什么需要 AMCL
2. 贝叶斯滤波与粒子表示
3. 运动模型（里程计模型）
4. 观测模型：似然域
5. 完整手算：两个粒子的权重
6. 重采样与有效粒子数
7. 自适应：KLD 采样
8. 全局定位、kidnapped 与恢复
9. 初始位姿
10. AMCL 的输出：位姿估计与 map→odom
11. 参数速查与车式小车的注意点
12. 失效模式与故障现象
13. 面试问答 / 要点速记 / 自测题 / 链接

---

## 1. 为什么需要 AMCL

[02 章](./02_车辆运动学与里程计.md)已经说明：航迹推算只增不减地积累误差。要想长期可靠，必须有**外部参考**。已知地图 + 激光扫描是一种很自然的参考：同一面墙，无论走多久，距离都是一样的。

但"用扫描匹配地图求位姿"不是一个有闭式解的问题：

- 观测（激光）与位姿的关系是复杂的非线性（要对地图做射线追踪/距离查询）；
- 后验分布可能是**多峰**的（长走廊里"在这里"和"在那里"看起来一样）；
- 开始时位姿可能完全未知。

EKF 假设单峰高斯，不适合这类问题。粒子滤波用大量样本去近似任意形状的分布，天然支持多峰，所以被选为地图定位的核心。

---

## 2. 贝叶斯滤波与粒子表示

### 2.1 贝叶斯滤波

状态 $\mathbf x_t=(x,y,\theta)$，控制/里程 $\mathbf u_t$，观测 $\mathbf z_t$（一帧激光），地图 $m$。信念（belief）$bel(\mathbf x_t)=p(\mathbf x_t\mid \mathbf z_{1:t},\mathbf u_{1:t},m)$。递推：

$$\overline{bel}(\mathbf x_t)=\int p(\mathbf x_t\mid\mathbf x_{t-1},\mathbf u_t)\,bel(\mathbf x_{t-1})\,d\mathbf x_{t-1}\quad\text{(预测)},$$
$$bel(\mathbf x_t)=\eta\,p(\mathbf z_t\mid\mathbf x_t,m)\,\overline{bel}(\mathbf x_t)\quad\text{(更新)}.$$

$\eta$ 是归一化常数。直觉：先按运动模型把"所有可能位置"推开（不确定度变大），再按观测的"符合程度"重新加权（不确定度变小）。卡尔曼滤波是它在线性高斯下的闭式解。

### 2.2 用粒子表示信念

用 $M$ 个带权样本近似：
$$bel(\mathbf x_t)\approx\{(\mathbf x_t^{[i]},w_t^{[i]})\}_{i=1}^{M},\qquad \sum_i w^{[i]}=1.$$

每个粒子是一个完整的位姿假设 $(x,y,\theta)$。MCL 的一个循环：

1. **采样（运动更新）**：对每个粒子，按运动模型 + 噪声，从 $p(\mathbf x_t\mid\mathbf x_{t-1}^{[i]},\mathbf u_t)$ 采样新位姿；
2. **加权（观测更新）**：$w^{[i]}\propto p(\mathbf z_t\mid\mathbf x_t^{[i]},m)$；
3. **重采样**：按权重抽取，让高权重粒子复制、低权重粒子消失，权重重置为相等。

```text
  粒子云 (M个假设)
      │  运动模型: 每个粒子按里程增量 + 噪声移动 (云变胖)
      ▼
  对每个粒子: 把激光束投到地图上, 查"是否对得上", 得到权重 w_i
      │
      ▼
  重采样: 权重高 → 复制; 权重低 → 淘汰   (云变瘦, 收敛到对得上的地方)
      │
      ▼
  输出: 加权均值 / 最大簇位姿 + 协方差
```

---

## 3. 运动模型（里程计模型）

### 3.1 里程计增量的分解

AMCL 把两次里程计位姿 $\bar{\mathbf x}_{t-1}=(\bar x,\bar y,\bar\theta)$ 与 $\bar{\mathbf x}_{t}=(\bar x',\bar y',\bar\theta')$ 之差，分解为"转—走—转"三段：

$$\delta_{rot1}=\mathrm{atan2}(\bar y'-\bar y,\ \bar x'-\bar x)-\bar\theta,\qquad \delta_{trans}=\sqrt{(\bar x'-\bar x)^2+(\bar y'-\bar y)^2},\qquad \delta_{rot2}=\bar\theta'-\bar\theta-\delta_{rot1}.$$

物理含义：先原地转 $\delta_{rot1}$ 面向运动方向，再直走 $\delta_{trans}$，最后转 $\delta_{rot2}$ 对到末端朝向。

### 3.2 加噪声并作用到每个粒子

每个粒子用**带噪声**的三段重新走一遍：
$$\hat\delta_{rot1}=\delta_{rot1}-\epsilon_1,\ \ \hat\delta_{trans}=\delta_{trans}-\epsilon_2,\ \ \hat\delta_{rot2}=\delta_{rot2}-\epsilon_3,$$
噪声方差：
$$\sigma_1^2=\alpha_1\delta_{rot1}^2+\alpha_2\delta_{trans}^2,\quad \sigma_2^2=\alpha_3\delta_{trans}^2+\alpha_4(\delta_{rot1}^2+\delta_{rot2}^2),\quad \sigma_3^2=\alpha_1\delta_{rot2}^2+\alpha_2\delta_{trans}^2.$$

$\alpha_1\ldots\alpha_4$ 是噪声参数（ROS `amcl` 里 `odom_alpha1`–`odom_alpha4`；全向模型多一个 `odom_alpha5`）：

| 参数 | 含义 |
|---|---|
| $\alpha_1$ | 旋转引起的旋转误差 |
| $\alpha_2$ | 平移引起的旋转误差 |
| $\alpha_3$ | 平移引起的平移误差 |
| $\alpha_4$ | 旋转引起的平移误差 |

新位姿：
$$x'=x+\hat\delta_{trans}\cos(\theta+\hat\delta_{rot1}),\ \ y'=y+\hat\delta_{trans}\sin(\theta+\hat\delta_{rot1}),\ \ \theta'=\theta+\hat\delta_{rot1}+\hat\delta_{rot2}.$$

> **运动模型的两个直觉**：噪声越大，云长得越"胖"（对里程计越不信，更依赖观测）；噪声过小，云太"瘦"，一旦里程计真实有误，所有粒子都偏，观测纠不回来（粒子贫化）。

### 3.3 车式小车的特别问题

**(a) 倒车。** 假设车沿 $\theta=0$ 倒车 0.5 m：位移向量 $(-0.5,0)$，则
$$\delta_{rot1}=\mathrm{atan2}(0,-0.5)-0=\pi,\quad \delta_{trans}=0.5,\quad \delta_{rot2}=0-\pi=-\pi.$$
也就是模型把倒车解释成"先原地转 180°，倒着前进，再转回来"。若 $\alpha_1=0.2$，则 $\sigma_1^2=0.2\pi^2+\alpha_2\cdot0.25\approx1.97$ rad²，标准差 1.4 rad——**粒子云会在方向上炸开**。ROS 的 `amcl` 在 `diff-corrected` 模型里对前后向做了**对称处理**：计算噪声时用 $\min(|\delta_{rot1}|,\ |\delta_{rot1}-\pi|)$ 代替 $|\delta_{rot1}|$，倒车就和前进被同等对待（此处为公开实现的做法，本项目的 `nav2_amcl` 使用差速里程模型并把 $\alpha$ 调大以容纳转向打滑，倒车靠联调时修正方向符号来保证对称）。若使用的实现没有这个处理，倒车时 AMCL 会明显退化——这是 CV 里"motion direction"问题在定位器内部的一种表现。

**(b) 没有"原地转"。** 里程模型的"转—走—转"只是数学分解，不要求车真的能原地转。但车式小车的真实运动是弧线，$\delta_{rot1}$、$\delta_{rot2}$ 在弯道里不为零；$\alpha$ 要按弧线运动的误差特性取，而不是直接沿用差速车的默认值。

**(c) 里程来源。** AMCL 的里程输入通常来自 TF `odom→base_link`，即 EKF 的输出；EKF 平滑，AMCL 的运动模型就更准。

**(d) 更新门限。** AMCL 只有在运动超过 `update_min_d`（平移）或 `update_min_a`（旋转）时才处理一帧激光——所以静止时 `map→odom` 不更新；车在慢速移动、阈值较大时，修正是**成块出现**的，这也是跳变来源之一。

---

## 4. 观测模型：似然域

### 4.1 要解决的问题

给定一个粒子位姿，一帧激光有 $K$ 束，每束得到一个障碍点。怎么给"这个位姿与地图有多吻合"打分？

**Beam 模型**（射线追踪）：对每个粒子的每束光，在地图里追踪射线得到"期望距离"，再与测量比较。精确但慢。

**似然域模型（likelihood field）**：不追踪射线，而是把每束光的**终点**投影到地图坐标，查询它到最近障碍物的距离 $d$。离障碍越近，越可能是真的。优点：只需预先计算一张"到最近障碍的距离图"（距离变换，distance transform），每束光 $O(1)$ 查表，快。缺点：不建模遮挡（忽略射线穿过障碍的事实）。AMCL 中默认和最常用的就是它。

### 4.2 每束光的概率

第 $k$ 束测得距离 $z_k$，对应方位角 $\phi_k$，粒子位姿 $(x,y,\theta)$，终点在地图系：
$$x_{z_k}=x+x_{sens}\cos\theta-y_{sens}\sin\theta+z_k\cos(\theta+\theta_{sens}+\phi_k),\quad y_{z_k}=\ldots$$
（先用激光的安装位姿 $x_{sens},y_{sens},\theta_{sens}$ 换算，再投射。**安装位姿错了，所有粒子的打分都会系统性偏。**）

查距离图得 $d_k$。单束概率是混合模型：
$$p(z_k)=z_{hit}\,\mathcal N(d_k;0,\sigma_{hit}^2)+\frac{z_{rand}}{z_{max}}\quad(+\ z_{short},\ z_{max}\text{ 项，likelihood field 常省略}).$$

含义：命中项（接近障碍的高斯）+ 随机噪声项（均匀）。ROS `amcl` 参数：`laser_z_hit`、`laser_z_rand`、`laser_sigma_hit`、`laser_max_range`。

### 4.3 整帧激光的权重

理论上各束独立，权重为乘积 $\prod_k p(z_k)$。但束与束之间并不独立（同一面墙），直接相乘会让权重过于尖锐（粒子贫化）。公开的 ROS `amcl` 实现采用**每束概率的立方再求和**的做法（`p += pz*pz*pz`，起平滑作用）；也只使用最多 `laser_max_beams` 束（如 30 束）均匀抽样，以控制计算量并降低束间相关带来的过度自信。

---

## 5. 完整手算：两个粒子的权重

**设定（教学数值）**：3 束激光，$\sigma_{hit}=0.2\ \mathrm m$，$z_{hit}=0.95$，$z_{rand}=0.05$，$z_{max}=10\ \mathrm m$（所以随机项 $=0.05/10=0.005$），单束概率 $p=z_{hit}\,e^{-d^2/(2\sigma^2)}+0.005$（这里省略高斯归一化常数，与 ROS 实现的写法一致）。每束最终累加 $p^3$。

**粒子 A（位姿对）**：3 束终点到最近障碍的距离 $d=(0.05,\ 0.10,\ 0.00)$ m。
- $d=0.05$：$e^{-0.0025/0.08}=e^{-0.03125}=0.9692$，$p=0.95\times0.9692+0.005=0.9257$，$p^3=0.7933$；
- $d=0.10$：$e^{-0.01/0.08}=e^{-0.125}=0.8825$，$p=0.8434$，$p^3=0.5999$；
- $d=0.00$：$p=0.95+0.005=0.955$，$p^3=0.8710$。
总权重 $w_A=0.7933+0.5999+0.8710=2.2642$。

**粒子 B（位姿错）**：$d=(0.5,\ 0.8,\ 0.3)$ m。
- $d=0.5$：$e^{-0.25/0.08}=e^{-3.125}=0.0439$，$p=0.0467$，$p^3=1.02\times10^{-4}$；
- $d=0.8$：$e^{-8}=3.35\times10^{-4}$，$p=0.00532$，$p^3\approx1.5\times10^{-7}$；
- $d=0.3$：$e^{-1.125}=0.3247$，$p=0.3134$，$p^3=0.0308$。
总权重 $w_B=0.0309$。

**归一化**（只考虑这两个粒子）：$\tilde w_A=\frac{2.2642}{2.2642+0.0309}=0.9865,\ \tilde w_B=0.0135$。A 的权重约是 B 的 73 倍。

**有效粒子数**：$N_{eff}=\dfrac{1}{\sum_i\tilde w_i^2}=\dfrac{1}{0.9732+0.0002}=1.03$。只剩约 1 个"有效"粒子——权重集中，应重采样。

---

## 6. 重采样与有效粒子数

### 6.1 为什么要重采样

不重采样，几次迭代后几乎所有权重集中在少数粒子上，其他粒子成为"僵尸"，计算浪费且分布失真（退化问题）。重采样把"权重差异"转换成"粒子数量差异"：权重大的被复制多份。

### 6.2 低方差（系统）重采样

只抽一个随机数 $r\sim U[0,1/M)$，然后以步长 $1/M$ 取 $M$ 个指针：$r,\ r+1/M,\ \ldots$，每个指针落在累积权重的哪一段，就选哪个粒子。比"每次独立抽"方差小，保持多样性。

> **示例手算**：$M=4$，权重 $[0.1,\ 0.2,\ 0.3,\ 0.4]$，累积 $[0.1,\ 0.3,\ 0.6,\ 1.0]$。取 $r=0.08$，指针 $0.08,\ 0.33,\ 0.58,\ 0.83$：
> - $0.08\le0.1$ → 粒子 1；
> - $0.33\in(0.3,0.6]$ → 粒子 3；
> - $0.58\in(0.3,0.6]$ → 粒子 3；
> - $0.83\in(0.6,1.0]$ → 粒子 4。
> 新粒子集 $\{1,3,3,4\}$（粒子 2 被淘汰，粒子 3 被复制），权重重置为 $1/4$。

### 6.3 何时重采样

不是每帧都重采样（频繁重采样会丢失多样性）。常用判据：$N_{eff}<M/2$，或每隔固定帧数（`resample_interval`）。

---

## 7. 自适应：KLD 采样

AMCL 中的 "A" 是自适应：粒子数 $M$ 随不确定度变化——云铺得开就多用粒子，收敛了就少用，节省计算。KLD（Kullback–Leibler divergence）采样：把状态空间分成网格（bin），数一数粒子占了 $k$ 个非空 bin，则需要的粒子数使"采样分布与真实后验的 KL 散度小于 $\varepsilon$ 的概率为 $1-\delta$"：

$$n=\frac{k-1}{2\varepsilon}\left\{1-\frac{2}{9(k-1)}+\sqrt{\frac{2}{9(k-1)}}\,z_{1-\delta}\right\}^3.$$

参数：`kld_err`（$\varepsilon$）、`kld_z`（$z_{1-\delta}$ 的分位），`min_particles`、`max_particles` 给上下界。

> **示例手算**：$k=10$ 个非空 bin，$\varepsilon=0.05$，$\delta=0.01$（$z_{0.99}=2.326$）：
> - $k-1=9$，$\dfrac{k-1}{2\varepsilon}=90$；
> - $\dfrac{2}{9\times9}=0.02469$，$\sqrt{0.02469}=0.1571$，乘 $z$：$0.3655$；
> - 括号内 $1-0.02469+0.3655=1.3408$，立方 $=2.410$；
> - $n=90\times2.410\approx217$ 个粒子。
> 若 $k=100$：$\frac{99}{0.1}=990$；$\frac{2}{9\times99}=0.002245$，开方 $0.0474$，乘 $2.326$ 得 $0.1102$；括号 $=1-0.0022+0.1102=1.1080$，立方 $1.360$，$n\approx1346$。**粒子分布越散，需要的粒子越多。**

---

## 8. 全局定位、kidnapped 与恢复

### 8.1 三种问题

| 问题 | 描述 | 对粒子滤波的要求 |
|---|---|---|
| 位置跟踪 | 已知初始位姿，持续跟踪 | 粒子集中，数量少即可 |
| 全局定位 | 初始位姿完全未知 | 粒子要均匀铺满空闲区域，逐步收敛 |
| 绑架（kidnapped） | 运行中被搬到别处 | 要有能"逃出当前假设"的机制 |

### 8.2 增强 MCL（augmented MCL）

跟踪时粒子都聚在错误位置，观测一直不吻合 → 权重普遍偏低。用平均权重的**短期与长期指数滑动平均**之比检测：

$$w_{slow}\leftarrow w_{slow}+\alpha_{slow}(w_{avg}-w_{slow}),\quad w_{fast}\leftarrow w_{fast}+\alpha_{fast}(w_{avg}-w_{fast}),$$
重采样时以概率 $p_{rand}=\max(0,\ 1-w_{fast}/w_{slow})$ 注入随机粒子。参数：`recovery_alpha_slow`、`recovery_alpha_fast`（默认 0 表示关闭）。

> **示例**：$w_{slow}=1.0$（长期平均），$w_{fast}=0.3$（最近突降）：$p_{rand}=1-0.3=0.7$，重采样时 70% 的新粒子是随机撒的。

### 8.3 全局定位服务

`/global_localization` 服务把所有粒子均匀撒到地图的空闲空间，让车边走边收敛。需要环境有足够的特征（非对称），否则会收敛到错误的对称位置。

### 8.4 工程含义

车式小车不能原地转，全局定位时"转一圈采集信息"要靠开弧线，空间限制时收敛慢。因此实际启动时通常**由人给定初始位姿**，AMCL 只做跟踪和小范围恢复。

---

## 9. 初始位姿

- 话题：`/initialpose`（`PoseWithCovarianceStamped`），RViz 中 "2D Pose Estimate" 发布；或参数 `initial_pose_x/y/a`、`initial_cov_xx/yy/aa`。
- 以给定位姿为中心、协方差为散布撒粒子。**协方差决定了"容错半径"**：给小了而位姿偏了 1 m，粒子云里根本没有正确假设，AMCL 收敛不回来（或收敛到墙里）；给大了，粒子分散，收敛慢，早期易被误导。
- 航向最敏感：位置偏几十厘米，激光匹配常常还能拉回；航向偏 30° 以上，往往拉不回。
- 自检：设初始位姿后，看激光点是否与地图墙壁重合（RViz 里叠加 `/scan`）。

> **注意**：初始位姿的**坐标系**是 `map`；在 RViz 点击时按 Fixed Frame 是 `map` 才对。

---

## 10. AMCL 的输出：位姿估计与 map→odom

### 10.1 估计值

AMCL 将粒子聚类，输出权重最大的簇的加权均值作为位姿（`/amcl_pose`，含协方差），并给出粒子云（`/particlecloud`）。协方差反映粒子云的胖瘦，可用来判断"当前定位是否可信"。

### 10.2 发布 map→odom

AMCL 不发布 `map→base_link`，而是用当前里程计位姿反推：
$$T_{map\,odom}=T_{map\,base}^{\text{AMCL}}\cdot\big(T_{odom\,base}\big)^{-1}.$$
并在时间戳上加 `transform_tolerance`，使变换在短时间内有效。手算例子见 [06 章](./06_EKF与AMCL协同_TF分工与连续状态.md)。

### 10.3 修正是"离散"的

AMCL 每处理一帧激光（受 `update_min_d/a` 门限）才更新一次，更新时位姿估计的变化会立刻体现在 `map→odom` 的值上。如果粒子云收敛得快，或簇换了，变化就是一个**台阶**——这是下游看到"位姿跳变"的主因之一，也是为什么要在下游做限速平滑（见 06 章）。

---

## 11. 参数速查与车式小车的注意点

以下为 ROS `amcl` 包的常见参数，默认值请以所用版本的文档为准。本项目取值约：`min_particles` 500、`max_particles` 3000、`pf_err` 0.05、`pf_z` 0.99、`laser_model_type: likelihood_field`、`max_beams` 60、`update_min_d` 0.15 m、`update_min_a` 0.2 rad、`resample_interval` 1、`alpha_slow` 0.001、`alpha_fast` 0.1、$\alpha_1\dots\alpha_4$ 取 0.15 / 0.10 / 0.20 / 0.10、`robot_model_type` 为差速模型。

| 参数 | 含义 | 调整建议 |
|---|---|---|
| `min_particles` / `max_particles` | 粒子数上下界 | 范围大/对称环境多一些；算力有限则少 |
| `kld_err` / `kld_z` | KLD 误差与置信 | 误差小→粒子多 |
| `update_min_d` / `update_min_a` | 触发更新的最小位移/转角 | 小→更新勤、算力高；大→修正更"块状" |
| `resample_interval` | 隔几次更新重采样一次 | 大→保多样性但收敛慢 |
| `transform_tolerance` | `map→odom` 的有效时长 | 太小→外推错误，太大→用旧变换 |
| `odom_model_type` | `diff` / `omni` / `diff-corrected` / `omni-corrected` | 车式没有专用模型，常选 corrected 的差速类并适当增大 α |
| `odom_alpha1…5` | 里程噪声 | 随里程质量调整；太小易贫化 |
| `laser_model_type` | `beam` / `likelihood_field` / `likelihood_field_prob` | 一般用似然域 |
| `laser_max_beams` | 使用的束数 | 少→快但信息少 |
| `laser_z_hit` / `z_rand` / `sigma_hit` | 观测模型混合与宽度 | `sigma_hit` 要与地图和激光精度匹配 |
| `laser_likelihood_max_dist` | 距离图最大膨胀距离 | |
| `recovery_alpha_slow/fast` | 增强 MCL | 非零开启 |
| `initial_pose_*`, `initial_cov_*` | 初始位姿与协方差 | 见第 9 节 |

**对车式小车的额外建议**：

1. 里程输入用 EKF 融合输出（平滑、含 IMU 角速度），而不是裸轮速里程；
2. 保证激光的安装位姿（`base_link→laser_link`）精确；
3. 检查倒车时的运动模型（对称处理）；
4. 地图分辨率与 `sigma_hit` 匹配：地图格 0.05 m、`sigma_hit` 0.2 m 属于常见量级（教学示例）；
5. 低速小车的 `update_min_d` 不宜过大，否则修正成块。

---

## 12. 失效模式与故障现象

| 现象 | 原因 | 判断与处理 |
|---|---|---|
| 长直走廊里沿走廊方向位置乱跑 | 激光无法约束沿墙方向的位置（几何退化） | 看粒子云沿走廊拉长；加入里程权重、增加特征（放置反光柱等）、EKF 融合保持连续 |
| 对称环境收敛到镜像位置 | 多峰 | 给准初始位姿；全局定位后检查；增加非对称特征 |
| 玻璃/镜面处位姿漂 | 激光无回波或错距 | 增大 `laser_z_rand`；使用滤波/地图标注 |
| 动态障碍多（人群）位姿被拉偏 | 观测与静态地图矛盾 | 减少束数、使用 `likelihood_field_prob` 等对离群点更稳的模型、提高 `z_rand` |
| 倒车时粒子云炸开 | 运动模型对倒车未对称处理 | 用 corrected 模型；检查 α |
| 转弯时位姿抖 | 激光时间戳/运动畸变 | 见 03 章，去畸变与时间对齐 |
| 启动后一直不收敛 | 初始位姿偏太多 / 协方差太小 | 重给初始位姿；加大 `initial_cov` |
| 粒子云收缩成一点后位姿偏（贫化） | α 太小、粒子数太少 | 增大 α、`min_particles` |
| `map→odom` 间歇性外推错误 | `transform_tolerance` 太小、CPU 过载 | 检查频率与负载 |

---

## 13. 面试问答 / 要点速记 / 自测题 / 链接

### 面试问答

**Q1. How does AMCL work?**

> AMCL keeps a set of pose hypotheses called particles. For every odometry update, each particle is moved by the motion model plus noise. Then each particle is weighted by how well the current LiDAR scan matches the map; I used the likelihood-field model, which looks up the distance from each beam endpoint to the nearest obstacle. Particles are resampled according to their weights, so the cloud converges to the poses that agree with the map. It is adaptive because the number of particles changes with KLD sampling.

*追问*：Why a particle filter rather than an EKF for map localisation? —— 观测模型强非线性且后验可能多峰，粒子可近似任意分布。

**Q2. What causes a pose jump in AMCL, and how do you reduce it?**

> The correction is applied in discrete steps when a scan is processed, so if the cloud converges or the dominant cluster changes, `map→odom` changes in a step. I reduce it by feeding AMCL smooth odometry from the EKF, making the motion noise reasonable, keeping particle count high enough, using a proper initial pose, and by not letting the controller depend directly on the map frame.
> 本项目的处理：AMCL 吃 EKF 的平滑 odom；$\alpha$ 放大；粒子数下限 500 保证簇不易切换；起点给定初始位姿；下游控制模块放在 odom 系，并在 AMCL 之后加限速平滑与门限（见 06 章）。

**Q3. What happens when the car reverses?**

> In the standard odometry motion model a backwards move looks like a turn of about 180° followed by forward motion, and the noise grows with that angle. If the implementation does not treat forward and backward symmetrically, the particle cloud spreads. So I check the odometry model setting and the sign of velocity in the whole chain.

**Q4. Walk me through the likelihood-field weighting.**

> For each particle I project each beam endpoint into the map, look up the distance to the nearest obstacle in a precomputed distance map, and turn that into a probability with a Gaussian plus a small uniform term. A particle whose beams end 5 cm from walls gets a weight about seventy times larger than one whose beams end 30–80 cm away.

### 本章要点速记

1. AMCL = 粒子滤波（运动采样 + 观测加权 + 重采样）+ KLD 自适应 + 增强恢复。
2. 运动模型：旋转—平移—旋转分解 + 4 个 α 噪声；倒车要对称处理，否则 $\delta_{rot1}\approx\pi$ 使云炸开。
3. 似然域：到最近障碍距离 → 高斯 + 均匀；预先算距离变换，$O(1)$ 查表。
4. 权重相乘过尖，实现里常做立方求和、限制束数。
5. $N_{eff}=1/\sum w_i^2$；低于 $M/2$ 重采样；低方差重采样保多样性。
6. KLD：分布越散粒子越多；$k=10,\varepsilon=0.05,\delta=0.01$ 约 217 个。
7. 恢复靠 $w_{fast}/w_{slow}$；全局定位把粒子撒满空闲区。
8. 输出 `map→odom`，修正离散，是位姿跳变的来源之一。
9. 初始位姿的航向与协方差最关键。

### 自测题

1. 为什么似然域模型比 beam 模型快？它忽略了什么？
2. 4 个粒子权重 $[0.4,0.3,0.2,0.1]$，求 $N_{eff}$。（答：$1/(0.16+0.09+0.04+0.01)=3.33$）
3. 用 $r=0.07$、$M=4$，权重 $[0.1,0.2,0.3,0.4]$ 做低方差重采样，结果？（答：指针 $0.07,0.32,0.57,0.82$，累积 $[0.1,0.3,0.6,1.0]$ → 粒子 1,3,3,4。）
4. 车倒车 1 m，推导 $\delta_{rot1}$、$\delta_{rot2}$，并说明对称处理的作用。
5. 长走廊里 AMCL 沿走廊方向不确定，EKF 的哪些信息可以弥补？

### 相关链接

- 上一章：[04 EKF 融合定位](./04_EKF融合定位.md)　下一章：[06 EKF 与 AMCL 协同](./06_EKF与AMCL协同_TF分工与连续状态.md)
- 基础：[../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md](../../02_基础知识/05_状态估计/05_粒子滤波与AMCL.md)
- 返回：[README](./README.md)
