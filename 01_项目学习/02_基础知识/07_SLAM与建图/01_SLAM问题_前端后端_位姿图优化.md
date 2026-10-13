# SLAM 问题：前端、后端与位姿图优化

> **一句话定位**：SLAM（Simultaneous Localization and Mapping，同时定位与建图）回答"机器人在一个没见过的环境里，怎样一边走一边画地图、一边知道自己在地图哪里"。本章讲清它的概率形式、前端/后端分工、位姿图优化（pose graph optimization）的完整推导与手算、占据栅格地图的更新，以及如何把它和 P5 里"已有地图上定位"的做法联系起来。

**本章学完你能：**

- 写出 SLAM 的概率问题 $p(x_{1:t},m\mid z_{1:t},u_{1:t})$，说出为什么"定位需要地图、建图需要定位"是个鸡生蛋问题；
- 对比 EKF-SLAM、粒子滤波 SLAM（FastSLAM / RBPF）与图优化 SLAM，并算出 EKF-SLAM 协方差的规模；
- 说明前端（数据关联、里程计、回环检测）与后端（全局优化）各做什么；
- 推导位姿图的误差函数、高斯—牛顿（Gauss–Newton）法方程 $H\Delta x=-b$，并手算一个 3 节点一维位姿图；
- 计算 SE(2) 位姿图的一条边的残差；
- 手算占据栅格的对数几率（log-odds）更新；
- 解释鲁棒核（Huber）、规范自由度（gauge freedom）、回环外点；
- 用英文回答 "What is SLAM / filter vs graph / why loop closure matters / how do you evaluate SLAM"。

**前置知识**：[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)（最小二乘、高斯—牛顿）、[坐标系、旋转矩阵、齐次变换](../03_运动学与坐标变换/01_坐标系_旋转矩阵_齐次变换.md)、[卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md)、[粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)。

## 目录

1. SLAM 要解决什么问题
2. 概率形式与符号
3. 三大流派：EKF、粒子滤波、图优化
4. 前端：里程计、数据关联、回环检测
5. 后端：位姿图优化推导
6. 手算一：一维位姿图
7. 手算二：SE(2) 一条边的残差
8. 鲁棒核与外点回环
9. 规范自由度与可观性
10. 地图表示与占据栅格更新（手算）
11. 典型激光 SLAM 系统一览
12. 评价指标：ATE 与 RPE
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
（补充）补充专题：边缘化、漂移量级与地图存储
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. SLAM 要解决什么问题

想象你被蒙眼放进陌生的大楼，只能摸着墙走，同时在纸上画平面图。你的每一步都靠"数步数"（里程计）估计位移，但步数会累积误差；你画的图依赖你认为自己在哪；你认为自己在哪又依赖图。这就是 SLAM：

- **定位（localization）**：已知地图，求位姿——P5 里的 AMCL 就是这样；
- **建图（mapping）**：已知位姿，求地图；
- **SLAM**：二者都未知，必须联合估计。

用途：机器人第一次进入新环境时建图；或在没有 GNSS 的室内、地下实时导航；也用于给后续的"已知地图定位"准备地图。

> **关键难点**：里程计误差随距离累积（漂移，drift）。若机器人回到曾经到过的地方（回环，loop closure），就能识别"我又回来了"，把累积的误差整体拉回来。**回环检测是消除漂移的唯一根本办法。**

## 2. 概率形式与符号

| 符号 | 含义 |
|---|---|
| $x_t$ | $t$ 时刻机器人位姿（2D 为 $(x,y,\theta)$，3D 为 6 自由度） |
| $m$ | 地图（栅格、点云、路标集合） |
| $u_t$ | 控制/里程计输入 |
| $z_t$ | 传感器观测（激光扫描、图像特征） |

**全 SLAM（full SLAM）**：估计整条轨迹加地图的后验
$$p(x_{1:t},m\mid z_{1:t},u_{1:t}).$$
**在线 SLAM（online SLAM）**：只估计当前位姿与地图，对过去位姿积分掉：$p(x_t,m\mid z_{1:t},u_{1:t})=\int\cdots\int p(x_{1:t},m\mid\cdot)\,dx_1\cdots dx_{t-1}$。

由贝叶斯与马尔可夫假设，全 SLAM 后验可分解为
$$p(x_{1:t},m\mid z,u)\ \propto\ p(x_0)\prod_t p(x_t\mid x_{t-1},u_t)\prod_t p(z_t\mid x_t,m).$$
前一项是**运动模型**，后一项是**观测模型**。取负对数，在高斯噪声下就是一个**非线性最小二乘**问题，这正是图优化的出发点。

## 3. 三大流派：EKF、粒子滤波、图优化

### 3.1 EKF-SLAM

把位姿和所有路标放进一个大状态 $[x_t,\,m_1,\dots,m_N]$，用 EKF 更新。2D 路标：状态维数 $3+2N$，协方差矩阵 $(3+2N)^2$。

**手算**：$N=1000$ 个路标：维数 2003，协方差 $2003^2=4.01\times10^6$ 个元素，双精度占 $4.01\times10^6\times8\,\text{B}\approx32$ MB；每次更新代价 $O(N^2)$。路标多了，计算和内存都爆。另外 EKF 的线性化一旦做错无法回头（一次线性化点定终身），容易不一致（inconsistency）。

### 3.2 粒子滤波 SLAM（FastSLAM、Rao–Blackwellized PF）

观察：给定整条轨迹，各路标估计彼此独立（条件独立）。所以可用**粒子**表示轨迹，每个粒子带自己的地图。栅格地图版本是 RBPF（Rao-Blackwellized particle filter）。著名的开源实现 gmapping 属于这类。代价：粒子数 × 地图大小；环境越大，每个粒子都存一张图，内存越高。

### 3.3 图优化（graph-based SLAM）

把轨迹上的位姿当作**节点**，位姿之间的相对测量（里程计、扫描匹配、回环）当作**边**，求使所有边的误差平方和最小的节点配置。现代主流 SLAM（Cartographer、LIO-SAM、ORB-SLAM 的后端等）几乎都是图优化或其滑动窗口变体。

| 方法 | 状态 | 优点 | 缺点 |
|---|---|---|---|
| EKF-SLAM | 当前位姿 + 路标 | 在线、概念简单 | $O(N^2)$，一次线性化 |
| RBPF | 粒子轨迹 + 每粒子地图 | 能处理多模态，可用栅格 | 粒子耗尽、内存大 |
| 图优化 | 全部（或窗口内）位姿 | 可反复线性化、稀疏高效、可全局一致 | 需要批量/增量求解器，回环外点敏感 |

## 4. 前端：里程计、数据关联、回环检测

SLAM 系统通常分两层：

```text
 传感器 ──> [ 前端 Front-end ] ──> 约束(边) ──> [ 后端 Back-end ] ──> 全局一致的轨迹+地图
            · 里程计/扫描匹配            · 位姿图/因子图优化
            · 数据关联                    · 鲁棒核、增量求解
            · 回环检测
```

### 4.1 里程计与扫描匹配

连续两帧激光点云做配准（ICP、NDT，见 [点云处理：滤波、配准、ICP](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)），得到相对位姿 $\Delta T_{i,i+1}$，成为一条"里程计边"。视觉则用特征匹配与几何求解（见 [视觉 SLAM 基础](./02_视觉里程计_特征_对极几何_PnP_BA.md)）。轮速里程计、IMU 预积分也可以作为边或初值。

### 4.2 数据关联

观测到的东西是不是以前见过的那个？错误关联是 SLAM 失败的头号原因。常用做法：门控（马氏距离，见 [卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md)）、描述子匹配、几何一致性检查（RANSAC）。

### 4.3 回环检测

判断"当前位置是否与历史某处相同"：

- 激光：全局描述子（如 Scan Context 一类，名称按公开文献，细节未核对）+ 扫描匹配验证；或基于子图的分支定界搜索（Cartographer 采用，细节未核对）；
- 视觉：词袋（Bag of Words）检索，再做几何验证（对极几何/PnP）。

**误检回环的后果极其严重**（把两个相似走廊当作同一处，地图被硬拉变形），所以必须"宁缺毋滥"：高阈值 + 几何验证 + 鲁棒核。

## 5. 后端：位姿图优化推导

### 5.1 问题

节点：位姿 $x_1,\dots,x_n$。边 $(i,j)$：测量到的相对位姿 $z_{ij}$（在 $i$ 的坐标系下 $j$ 的位姿），其信息矩阵（协方差的逆）$\Omega_{ij}$。

给定估计 $x_i,x_j$，**预测的**相对位姿为 $\hat z_{ij}(x_i,x_j)$，误差
$$e_{ij}=e(x_i,x_j,z_{ij})=\hat z_{ij}(x_i,x_j)\ominus z_{ij}.$$
（$\ominus$ 表示在位姿群上的"差"，对 SE(2)/SE(3) 要用李群的 log 映射，对一维就是普通减法。）

最大后验估计等价于
$$x^*=\arg\min_x\ F(x),\qquad F(x)=\sum_{(i,j)}e_{ij}^\top\Omega_{ij}\,e_{ij}.$$
这就是**位姿图优化**。权重 $\Omega$ 大的边（更可信的测量）更被满足。

### 5.2 高斯—牛顿

对当前估计 $x$，设微小增量 $\Delta x$，一阶展开 $e_{ij}(x+\Delta x)\approx e_{ij}+J_{ij}\Delta x$。代入：
$$F(x+\Delta x)\approx\sum\big(e_{ij}^\top\Omega e_{ij}+2e_{ij}^\top\Omega J_{ij}\Delta x+\Delta x^\top J_{ij}^\top\Omega J_{ij}\Delta x\big).$$
令对 $\Delta x$ 的导数为 0：
$$H\,\Delta x=-b,\qquad H=\sum J_{ij}^\top\Omega_{ij}J_{ij},\quad b=\sum J_{ij}^\top\Omega_{ij}e_{ij}.$$
更新 $x\leftarrow x\boxplus\Delta x$，重复到收敛。若 $H$ 病态或初始化差，用 **Levenberg–Marquardt**：解 $(H+\lambda I)\Delta x=-b$（$\lambda$ 大时像梯度下降，更稳）。

### 5.3 稀疏结构

每条边只依赖两个节点，所以 $J_{ij}$ 只有对应 $x_i,x_j$ 两块非零，$H$ 是**块稀疏**的：只有"有边相连"的节点对才有非零块。里程计链给出块三对角结构，回环给出远离对角线的块。稀疏 Cholesky 分解（CHOLMOD）或增量方法（iSAM2 一类）能高效求解。这也是为什么图优化比 EKF 的稠密协方差高效得多。

## 6. 手算一：一维位姿图

场景（示例）：机器人沿直线走。节点 $x_0=0$（固定为原点）、$x_1$、$x_2$。

- 里程计边：$x_1-x_0=1.0$，$x_2-x_1=1.0$（轮子计数；但存在比例误差）；
- 回环边：回到参考点附近，激光匹配给出 $x_2-x_0=1.8$。

纯里程计会得到 $x_2=2.0$，与回环的 1.8 矛盾（矛盾 0.2，是累积漂移）。设三条边权重都为 1：

残差：$r_1=x_1-x_0-1.0$，$r_2=x_2-x_1-1.0$，$r_3=x_2-x_0-1.8$。以 $x=[x_1,x_2]^\top$ 为变量（$x_0=0$）：
$$A=\begin{bmatrix}1&0\\-1&1\\0&1\end{bmatrix},\quad c=\begin{bmatrix}1.0\\1.0\\1.8\end{bmatrix},\quad r=Ax-c.$$
$$H=A^\top A=\begin{bmatrix}2&-1\\-1&2\end{bmatrix},\quad A^\top c=\begin{bmatrix}0\\2.8\end{bmatrix}.$$
解 $Hx=A^\top c$：行列式 $4-1=3$，$H^{-1}=\frac13\begin{bmatrix}2&1\\1&2\end{bmatrix}$，$x=\frac13[2\cdot0+1\cdot2.8,\ 1\cdot0+2\cdot2.8]=[0.9333,\ 1.8667]$。

残差：$r=[-0.0667,-0.0667,+0.0667]$，总代价 $F=3\times0.0667^2=0.0133$（Python 复核）。**直觉**：0.2 的矛盾被均匀摊给了三条边，每条边各让一点（0.0667）。

**加权**：若回环边更可信，权重取 4（信息矩阵放大）：$H=\begin{bmatrix}2&-1\\-1&5\end{bmatrix}$，$A^\top Wc=[0,\ 8.2]$，解得 $x=[0.9111,\ 1.8222]$，回环边残差缩到 0.022，里程计边各被拉到 $-0.089$。**可见权重决定"相信谁"**。

在线性问题里高斯—牛顿一步到位；在 SE(2)/SE(3) 非线性问题里需要迭代若干次，并且线性化点要从可靠的初值（里程计积分）开始，初值太差可能陷入局部极小。

## 7. 手算二：SE(2) 一条边的残差

用常见的 2D 位姿图边误差（与 g2o 的 SE2 边等价于小角度形式）：位姿 $x_i=(x_i,y_i,\theta_i)$，测量 $z_{ij}=(\Delta x,\Delta y,\Delta\theta)$ 是 $j$ 在 $i$ 坐标系下的位姿。误差
$$e=\begin{bmatrix}R_i^\top(t_j-t_i)-\Delta t\\ \theta_j-\theta_i-\Delta\theta\end{bmatrix},\quad R_i=\begin{bmatrix}\cos\theta_i&-\sin\theta_i\\ \sin\theta_i&\cos\theta_i\end{bmatrix}$$
（角度差要规整到 $(-\pi,\pi]$。）

**例 1**：$x_i=(1,0,0)$，$x_j=(2.1,0.05,0.04)$，测量 $(1.0,0,0)$。$R_i=I$，$R_i^\top(t_j-t_i)=(1.1,0.05)$，$e=(0.1,0.05,0.04)$。

**例 2**（有旋转）：$x_i=(1,0,\pi/2)$（朝向 $+y$），$x_j=(1,1,\pi/2)$，测量 $(1,0,0)$（前进 1 m）。$t_j-t_i=(0,1)$，$R_i^\top=\begin{bmatrix}0&1\\-1&0\end{bmatrix}$，乘得 $(1,0)$，减去 $\Delta t=(1,0)$ 得 0，$e=(0,0,0)$（Python 复核）。**要点**：误差是在 $i$ 的坐标系里度量的，不是世界系，这样同样的"向前走 1 m"在任何朝向下都一致。

## 8. 鲁棒核与外点回环

最小二乘对外点（outlier）极度敏感：一条错误的回环边残差 3.0，代价 $9$，会主导优化。**鲁棒核（robust kernel）**把大残差的惩罚降到线性。Huber 核（阈值 $\delta$）：
$$\rho(r)=\begin{cases}\tfrac12r^2,&|r|\le\delta\\ \delta(|r|-\tfrac12\delta),&|r|>\delta\end{cases}$$
**手算**：$\delta=1$：$r=0.5$ 时 $\rho=0.125$（与二次相同）；$r=3$ 时 $\rho=2.5$（二次将是 4.5）。其他：Cauchy、Geman–McClure、switchable constraints / 一致性检验（pairwise consistency）等对回环外点更激进。

## 9. 规范自由度与可观性

位姿图中**只有相对测量**，所以整张图可以整体平移旋转而代价不变：$H$ 奇异（秩亏）。解决：固定第一个节点（锚定），或加先验边。2D 秩亏 3，3D 秩亏 6。

**可观性**：没有绝对参照时，全局位置和朝向不可观，只有相对结构可观。有 GNSS 或已知路标时才可得到全局坐标（引入绝对位置因子）。

## 10. 地图表示与占据栅格更新（手算）

### 10.1 常见地图

| 地图 | 内容 | 用途 |
|---|---|---|
| 占据栅格（occupancy grid） | 每格被占据的概率 | 2D 导航、代价地图 |
| 点云/体素地图 | 3D 点 | 3D 建图、重定位 |
| 特征/路标地图 | 稀疏 3D 点+描述子 | 视觉 SLAM 定位 |
| 子图（submap） | 局部栅格拼成 | Cartographer：局部稠密、全局靠子图相对位姿 |
| 拓扑/语义地图 | 节点与语义 | 任务级规划 |

### 10.2 对数几率更新

每个栅格维护对数几率 $l=\ln\frac{p}{1-p}$。独立观测可直接相加（贝叶斯更新在 log-odds 下是加法）：$l_t=l_{t-1}+l_{meas}-l_0$。设先验 $p_0=0.5$（$l_0=0$），"击中"（beam 终点）传感器模型 $p=0.7\Rightarrow l_{hit}=\ln\frac{0.7}{0.3}=0.847$，"穿过"（beam 经过的空闲格）$p=0.4\Rightarrow l_{miss}=\ln\frac{0.4}{0.6}=-0.405$。

**手算**一个格子依次观测 hit, hit, miss, hit：

| 步 | 观测 | $l$ | $p=\sigma(l)$ |
|---|---|---|---|
| 1 | hit | 0.847 | 0.700 |
| 2 | hit | 1.695 | 0.845 |
| 3 | miss | 1.289 | 0.784 |
| 4 | hit | 2.136 | 0.894 |

（Python 复核。）通常还对 $l$ 做上下限截断（如 $[-4,4]$），让动态物体消失后地图可以"改口"。栅格是否算障碍用阈值判定（如 $p>0.65$，取值因实现而异，未核对）。射线遍历用 Bresenham 算法逐格更新。

## 11. 典型激光 SLAM 系统一览

以下仅作概览，系统名称与大致做法来自公开资料，具体参数与细节**未核对**，面试时只讲思想：

- **gmapping**：RBPF 栅格 SLAM，适合小场景、需要里程计；
- **Cartographer**：局部子图扫描匹配 + 分支定界回环 + 稀疏位姿图（Ceres）优化，支持 2D/3D；
- **slam_toolbox**（Nav2 常用的建图工具）：基于位姿图，支持序列化地图、继续建图、纯定位模式；
- **LOAM / LIO-SAM**：3D 激光，特征点（边/面）配准 + IMU 紧耦合 + 因子图（见 [激光 SLAM 后续章节计划](../结构说明与扩展计划.md)）；
- **ORB-SLAM 系列**：特征点法视觉 SLAM（见 [视觉里程计](./02_视觉里程计_特征_对极几何_PnP_BA.md)）。

## 12. 评价指标：ATE 与 RPE

- **ATE（Absolute Trajectory Error）**：估计轨迹与真值先对齐（刚体变换，视觉单目还要尺度），再算均方根：$ATE_{rmse}=\sqrt{\frac1N\sum\|p_i^{est}-p_i^{gt}\|^2}$。反映全局一致性。**示例**：四个位置误差分别 0.1、0.2、0.1、0.2 m，则 $\sqrt{(0.01+0.04+0.01+0.04)/4}=\sqrt{0.025}=0.158$ m。
- **RPE（Relative Pose Error）**：固定时间间隔（或距离）上相对位姿的误差，反映局部漂移率。
- 其他：回环召回率与精度、地图一致性（目视）、运行时间与内存。

没有真值时，可用回到起点的闭合差（closure error）近似评价漂移。

## 13. 在本项目里怎么用

- **P5**：CV 的表述是"在地图上做 EKF/AMCL 定位"，即**使用已有栅格地图的定位**。SLAM 是它的"上游"：地图可由激光 SLAM（如 slam_toolbox 一类）建出后保存，再供 AMCL 使用。面试若被问"地图怎么来的"，可以如实说明建图是典型的 2D LiDAR SLAM 流程，并讲清原理；不要说成自己实现了完整 SLAM，除非有依据（见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)）。
- **粒子滤波的联系**：AMCL 与 RBPF-SLAM 同属粒子滤波，区别在于 AMCL 的地图固定、粒子只表示位姿，故计算轻得多（见 [粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)）。
- **漂移与连续状态**：AMCL 修正造成的 `map→odom` 跳变，与位姿图回环造成的轨迹"拉回"在思想上同源：局部连续、全局一致，需要两层坐标系分工（见 [时间同步与坐标系](../06_状态估计/06_时间同步_坐标系一致性与数据有效性.md)）。
- **P6**：多机器人共享地图与状态时，每台机器人的局部位姿图如何合并，是多机 SLAM 的问题；平台层只需提供统一时间戳与坐标系。

## 14. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 走廊长了被"压短" | 激光扫描匹配在退化环境（长走廊）沿走廊方向不可观 | 融合轮速/IMU，限制该方向更新 |
| 地图重影、墙壁双层 | 回环没闭合或位姿漂移后叠加 | 增加回环；检查里程计标定 |
| 地图被硬拉扭曲 | 误检回环 | 提高回环阈值，加几何验证与鲁棒核 |
| 优化不收敛 | 初值差、尺度不一致、规范未固定 | 用里程计初值，固定首节点，LM 阻尼 |
| 时间戳错位使地图模糊 | 激光与里程计未同步，运动畸变未去除 | 时间同步、按运动做畸变校正 |
| 动态物体留下"鬼影" | 栅格没有衰减或上限 | log-odds 截断，动态点过滤 |
| 实时性差 | 子图/节点太多、全图优化 | 滑动窗口、增量求解、子图 |

## 15. 面试怎么问、怎么答

**Q1. What is SLAM and why is it hard?**

> SLAM estimates the robot trajectory and the map at the same time. It is hard because localisation needs a map and mapping needs the pose, odometry drifts without bound, and data association errors can corrupt the whole map. Loop closure is what bounds the drift.

中文：SLAM 同时估计轨迹与地图；难在互相依赖、里程计无界漂移、数据关联错误会毁掉全图；回环限制漂移。追问：回环误检怎么办？（几何验证、鲁棒核、一致性检验。）

**Q2. Filter-based versus graph-based SLAM?**

> A filter keeps only the current state and marginalises the past, so linearisation points are frozen and EKF-SLAM costs O(N squared). Graph-based SLAM keeps the whole trajectory as nodes and relative measurements as edges, relinearises repeatedly, and exploits the sparsity of the problem. That is why modern systems use graph optimisation with a sliding window or incremental solver.

中文：滤波只保留当前状态，线性化点被冻结，EKF-SLAM 为 $O(N^2)$；图优化保留整条轨迹，可反复线性化并利用稀疏性，因此现代系统多用图优化。

**Q3. Walk me through pose graph optimisation.**

> Nodes are poses and edges are relative measurements with information matrices. For each edge I compute the error between the predicted and measured relative pose, and minimise the sum of squared, information-weighted errors with Gauss-Newton or Levenberg-Marquardt. The normal equation H dx = -b is sparse, and I fix the first node to remove the gauge freedom.

中文：节点是位姿，边是带信息矩阵的相对测量；每条边算预测与测量相对位姿之差，用 GN/LM 最小化加权平方和；$H$ 稀疏，固定首节点消去规范自由度。

**Q4. In your navigation project, did you build the map with SLAM?**（诚实回答模板）

> My work was the localisation, planning and tracking on a given occupancy grid: EKF fusion, AMCL, Hybrid A star, TEB and MPC. The map itself comes from a standard 2D LiDAR SLAM workflow; I understand how it works, graph optimisation with loop closure, but I want to be clear that building the SLAM system was not my contribution.

中文：我负责在给定栅格地图上的定位、规划与跟踪；地图来自标准 2D LiDAR SLAM 流程，我理解其原理（图优化+回环），但 SLAM 系统本身不是我的贡献。（仅在你确实没有做过建图时使用。）

**Q5. How do you evaluate a SLAM result?**

> With ground truth I use absolute trajectory error after alignment, and relative pose error for local drift. Without ground truth I look at loop closure error, map consistency such as double walls, and repeatability across runs.

中文：有真值用对齐后的 ATE 与反映局部漂移的 RPE；没有真值看闭合差、地图一致性（是否重影）、多次重复性。

## 补充专题：边缘化、漂移量级与地图存储

### 边缘化（marginalization）：滑动窗口怎样"忘掉"旧位姿

长时间运行时，位姿图会无限增长。滑动窗口 SLAM 的做法是把最老的节点从优化变量里移除，但**不能简单丢弃**它带来的约束，否则信息白白丢失。正确做法是**边缘化**：对高斯分布的信息矩阵 $H$ 使用 Schur 补，把被移除变量 $m$ 的信息折算到保留变量 $k$ 上：
$$H_{kk}^{new}=H_{kk}-H_{km}H_{mm}^{-1}H_{mk}.$$

**手算**：三个节点 $x_0,x_1,x_2$ 串联，两条权重为 1 的里程计边 $x_1-x_0$ 与 $x_2-x_1$。信息矩阵（未固定规范）
$$H=\begin{bmatrix}1&-1&0\\-1&2&-1\\0&-1&1\end{bmatrix}.$$
边缘化 $x_1$：保留 $(x_0,x_2)$。$H_{mm}=2$，$H_{km}=[-1,-1]^\top$。
$$H^{new}_{kk}=\begin{bmatrix}1&0\\0&1\end{bmatrix}-\frac12\begin{bmatrix}-1\\-1\end{bmatrix}\begin{bmatrix}-1&-1\end{bmatrix}=\begin{bmatrix}0.5&-0.5\\-0.5&0.5\end{bmatrix}.$$
这正是一条连接 $x_0,x_2$、权重 0.5 的边（Python 复核）。直觉：两段独立的里程计串联，方差相加，信息（方差的倒数）减半。**边缘化不会丢信息，但会让原本稀疏的 $H$ 变稠密**（被消去节点的所有邻居彼此相连，称为"填充"，fill-in），所以滑动窗口里要控制窗口大小，并注意**线性化点固定**带来的一致性问题（FEJ，first-estimate Jacobian，思想概述）。

### 里程计漂移的量级感

用两个简单例子建立直觉：

1. **比例误差**：轮式里程计比例误差 1%，走 200 m，累积距离误差 $200\times0.01=2$ m；
2. **航向误差的放大**：航向误差 $1^\circ$，沿该方向走 50 m，横向偏差 $50\tan1^\circ=0.87$ m。

所以**航向误差是最危险的**，它随距离线性放大成位置误差。这就是为什么 IMU/陀螺或磁力计对里程计至关重要，也是 P5 要对时间、坐标系、运动方向不一致引起的漂移做处理的原因。回环检测时发现的"闭合差"，往往 90% 来自航向漂移。

### 地图存储的规模

2D 占据栅格：分辨率 0.05 m，场地 100 m × 100 m，格数 $2000\times2000=4\times10^6$，每格 1 字节（int8 概率）约 4 MB，完全可放内存；若是 3D 体素 $0.1$ m，场地 $100\times100\times10$ m，体素数 $10^8$，但真实占据率通常只有约 2%，用哈希存稀疏体素，每个体素约 16 字节，则 $2\times10^6\times16=32$ MB。两者规模差别说明：**2D 导航地图轻量，3D 地图必须稀疏化/分块**。

### 回环检测的精确率与召回率

回环检测是分类问题：真回环为正例。**精确率**（precision，报告的回环里有多少是真的）比**召回率**（recall，真回环里被找到多少）更重要，因为一个错误回环可以毁掉整张地图，而漏检只是损失一次纠正机会。工程做法：把阈值调到精确率接近 100%，再用多帧一致性（连续 $n$ 帧都指向同一历史位置）补回召回率。评估时画精确率-召回率曲线，并指出工作点。

### 最小工作流（面试可以口述）

1. 激光/视觉前端输出每帧相对位姿，加入位姿图；
2. 关键帧选取：位移超过 0.3~0.5 m 或旋转超过 $10^\circ$ 才添加（经验取值，未核对）；
3. 回环候选：位置距离近但时间间隔长；用扫描匹配验证并给出相对位姿；
4. 后端优化（带鲁棒核），输出校正后的轨迹；
5. 由校正后的轨迹重新渲染占据栅格并保存；
6. 导航时用保存的地图做定位（AMCL），与 P5 的流程衔接。

### 补充：为什么位姿图比滤波更适合长时间运行

用一句话总结：滤波把过去压缩成当前的均值和协方差，之后无法修改；位姿图保留过去的相对测量，回环到来时可以重新解释整段轨迹。这相当于把"事后诸葛亮"变成了算法的一部分，代价是内存随时间增长，需要边缘化、关键帧筛选、子图等手段限制规模。

## 16. 本章要点速记

1. SLAM = 联合估计轨迹与地图；回环是消除漂移的关键。
2. 全 SLAM 后验 = 运动模型 × 观测模型；取负对数得非线性最小二乘。
3. EKF-SLAM 协方差 $O(N^2)$，1000 路标约 32 MB；RBPF 每粒子一图。
4. 图优化：节点位姿、边相对测量，$F=\sum e^\top\Omega e$。
5. 高斯—牛顿 $H\Delta x=-b$，$H=\sum J^\top\Omega J$ 块稀疏；LM 加阻尼。
6. 只有相对测量时存在规范自由度（2D 秩亏 3），需固定首节点。
7. 权重（信息矩阵）决定相信哪条边；Huber 核限制外点影响。
8. 栅格更新在 log-odds 下是加法，要截断上下限。
9. 评价：ATE（全局）、RPE（局部漂移）、闭合差。
10. P5 是"已有地图上的定位"，SLAM 是其上游。

## 17. 自测题

1. 1000 个 2D 路标的 EKF-SLAM 协方差有多少元素？（$(3+2000)^2\approx4.01\times10^6$。）
2. 一维图 $x_1-x_0=1,\ x_2-x_1=1,\ x_2-x_0=1.8$，权重全 1，解是多少？（$[0.933,1.867]$。）
3. 为什么位姿图的 $H$ 是奇异的？怎么解决？（只有相对测量→整体平移旋转不变；固定首节点或加先验。）
4. Huber 核在 $\delta=1$、$r=3$ 时代价是多少？（$2.5$。）
5. 占据栅格某格 $l=1.2$，再来一次 miss（$-0.405$），$p$ 变成多少？（$l=0.795$，$p=1/(1+e^{-0.795})=0.689$。）
6. 长走廊里激光 SLAM 为何会"压短"？（沿走廊方向几何特征退化，扫描匹配不可观。）

## 18. 相关链接

- 下一章：[视觉里程计：特征、对极几何、PnP、BA](./02_视觉里程计_特征_对极几何_PnP_BA.md)
- 相关：[粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)、[里程计与传感器融合](../06_状态估计/04_里程计与传感器融合.md)、[点云处理：滤波、配准、ICP](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[07_SLAM与建图 README](./README.md)
