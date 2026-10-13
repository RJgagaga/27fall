# 激光 SLAM：LOAM、LIO-SAM、Cartographer 与 gmapping/slam_toolbox

> **一句话定位**：激光 SLAM 用激光雷达直接测得的距离，通过"扫描匹配 + 后端优化"同时估计机器人轨迹和地图。本章把 2D 激光（gmapping、Cartographer、slam_toolbox）和 3D 激光（LOAM、LIO-SAM）放在同一条主线上讲：前端怎么把两帧点云对齐，后端怎么把误差摊平，回环怎么消除漂移。它是面试里"你用过哪种 SLAM、它们有什么区别"这类问题的标准答案库。

**本章学完你能：**

- 说出激光 SLAM 的统一结构：运动畸变补偿 → 特征/栅格 → 扫描匹配 → 关键帧/子图 → 回环 → 后端优化；
- 手算 LOAM 的曲率特征，区分边缘点与平面点，写出点到线、点到面残差，并做一步高斯-牛顿；
- 解释运动畸变（去畸变）为什么必须做，并手算一个点的补偿；
- 说明 LIO-SAM 的因子图由哪些因子构成，为什么用 IMU 预积分去畸变；
- 解释 Cartographer 的子图（submap）、分支定界（branch and bound）扫描匹配与回环的关系；
- 对比 gmapping（粒子滤波）、slam_toolbox（图优化）、Cartographer 的适用场景；
- 用英文回答 "How does LOAM extract features / What is the difference between filter-based and graph-based SLAM / Why scan-to-map instead of scan-to-scan"。

**前置知识**：[SLAM 问题：前端、后端与位姿图优化](./01_SLAM问题_前端后端_位姿图优化.md)、[点云处理：滤波、配准与 ICP](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)、[粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)、[传感器：IMU、编码器、激光雷达](../06_状态估计/01_传感器_IMU_编码器_激光雷达.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。

## 目录

1. 激光 SLAM 要解决什么问题
2. 符号与单位
3. 统一结构：一张图看懂所有激光 SLAM
4. 运动畸变与去畸变（含手算）
5. LOAM：曲率特征（含手算）
6. LOAM：点到线、点到面残差与一步高斯-牛顿（含手算）
7. LOAM 的双频结构与后续改进
8. LIO-SAM：因子图与紧耦合
9. gmapping：Rao-Blackwellized 粒子滤波
10. Cartographer：子图与分支定界（含手算）
11. slam_toolbox：Karto 图优化与终身建图
12. 系统对比表
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 激光 SLAM 要解决什么问题

激光雷达（LiDAR）每个扫描周期给出一圈"方位角 + 距离"。2D 激光是一个平面内的几百到上千个点，3D 激光是几万到十几万个点。把这些点看作"机器人在某位姿下看到的环境剖面"，激光 SLAM 就是回答两个互相依赖的问题：

1. **我在哪里？**——已知地图，把当前扫描和地图对齐，得到位姿；
2. **地图长什么样？**——已知位姿，把扫描放到全局坐标，累积成地图。

两个问题互为前提，所以叫"同时定位与建图"。激光 SLAM 与视觉 SLAM 的区别很实际：激光直接给度量距离，没有尺度问题，对光照不敏感，但数据稀疏、缺少纹理信息，在长走廊、空旷广场这类几何退化场景里会"滑"。

```text
扫描 k ──去畸变──> 特征/栅格 ──匹配(对齐到地图)──> 位姿 T_k
                                      │
              关键帧/子图 <────────────┘
                   │
              回环检测 ──> 后端图优化 ──> 全局一致的轨迹与地图
```

> **提示**：面试里最容易混的是"里程计（odometry）"和"SLAM"。里程计只管相邻帧，漂移不可避免；SLAM 多了"地图"和"回环"，才有机会把漂移拉回来。

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $T_k=(R_k,t_k)$ | 第 $k$ 帧雷达在世界系下的位姿 | $R\in SO(3)$（2D 为 $SO(2)$），$t$ 单位 m |
| $p_i$ | 第 $i$ 个激光点（雷达系） | m |
| $c_i$ | 点 $i$ 的局部曲率 | 无量纲 |
| $S$ | 点 $i$ 同一扫描线上的邻域点集合 | 左右各取若干点 |
| $d_\varepsilon,d_\pi$ | 点到边缘线、点到平面的距离残差 | m |
| $\mathbf n$ | 平面单位法向量 | 无量纲 |
| $v,\omega$ | 扫描期间的线速度、角速度 | m/s，rad/s |
| $s\in[0,1]$ | 点在一次扫描中的相对时刻 | 0 为扫描起点，1 为终点 |
| $N_{\text{eff}}$ | 粒子滤波有效粒子数 | 无量纲 |

坐标约定：右手系，$x$ 向前，$y$ 向左，$z$ 向上；旋转角逆时针为正，与 ROS REP-103 一致。

## 3. 统一结构：一张图看懂所有激光 SLAM

不管名字叫什么，激光 SLAM 都由同样五块拼成。区别只在每一块的具体选择：

| 模块 | 作用 | LOAM | LIO-SAM | Cartographer | gmapping | slam_toolbox |
|---|---|---|---|---|---|---|
| 去畸变 | 消除扫描期间运动造成的变形 | 匀速假设插值 | IMU 积分 | IMU/里程计外推 | 里程计 | 里程计 |
| 前端匹配 | 当前扫描对齐到参考 | 特征点对局部地图 | 特征点对局部地图 | 扫描对子图（相关性） | 扫描对每个粒子的地图 | 扫描对局部扫描（相关性 + Karto） |
| 参考表示 | 与谁匹配 | 特征体素地图 | 滑窗关键帧特征 | 概率栅格子图 | 每粒子一张栅格 | 局部扫描集合 |
| 后端 | 全局一致 | 无（原版） | 因子图（iSAM2） | 稀疏位姿调整 SPA | 无（靠粒子多样性） | 稀疏位姿图（Ceres） |
| 回环 | 消除长期漂移 | 无（原版） | 距离候选 + ICP | 分支定界匹配 | 无显式回环 | 扫描匹配回环 |

> **注意**：以上对各系统内部细节的概括来自我对公开论文与开源代码结构的理解，具体参数名、版本差异和默认值我没有逐条核对源码，面试时请说"根据我对论文的理解"，不要背默认参数。

## 4. 运动畸变与去畸变（含手算）

### 4.1 为什么会有畸变

机械旋转式激光雷达转一圈需要一个扫描周期（典型 10 Hz，即 0.1 s，具体型号以数据手册为准）。如果机器人在这 0.1 s 内移动或转动，同一圈里不同时刻测得的点其实站在不同的位姿上，把它们简单地"当成同一时刻"拼成点云，墙就会变弯。机器人速度越快、角速度越大，畸变越明显。

### 4.2 去畸变的做法

设扫描起点时刻位姿为参考系。点 $i$ 的相对时刻为 $s_i\in[0,1]$，扫描期间的运动用匀速模型近似：

$$
\theta_i=\omega\,s_i T,\qquad \mathbf t_i=\mathbf v\,s_i T
$$

其中 $T$ 为扫描周期。第 $i$ 个点补偿到扫描起点坐标系：

$$
p_i^{\text{start}}=R(\theta_i)\,p_i+\mathbf t_i
$$

LOAM 用雷达上一次估计的位姿增量做线性插值；LIO-SAM 用 IMU 积分得到更准的扫描内姿态；2D 系统（Cartographer、slam_toolbox）通常用里程计或 IMU 外推。

### 4.3 手算

示例（非项目实测）：$v=1\,$m/s 沿 $x$ 方向，$\omega=0.5\,$rad/s，$T=0.1\,$s。某点在 $s=0.8$ 时刻测得，雷达系坐标 $(5,\,2)$。

- 时间 $t=sT=0.08\,$s；
- 转角 $\theta=\omega t=0.04\,$rad；平移 $\mathbf t=(0.08,\,0)$；
- $\cos0.04=0.99920,\ \sin0.04=0.03999$；
- $x'=5\times0.99920-2\times0.03999+0.08=4.9960-0.0800+0.08=4.996$；
- $y'=5\times0.03999+2\times0.99920=0.19995+1.9984=2.198$。

结果 $(4.996,\,2.198)$（Python 复核：`4.99602186, 2.19834688`）。看起来只改了 0.004 m 和 0.198 m——$y$ 方向的 0.198 m 是由 0.04 rad 的转角在 5 m 远处放大出来的（弧长约 $r\theta=0.2$ m）。这就是为什么**角速度对畸变的影响比线速度更大，且随距离放大**。

> **面试要点**：被问"为什么要去畸变"时，给一个量级：在 5 m 处，仅 2.3 度（0.04 rad）的扫描内转动就带来约 20 cm 的点位偏差，足以破坏特征匹配。

## 5. LOAM：曲率特征（含手算）

### 5.1 问题

3D 激光一帧有几万个点，直接 ICP 太慢。LOAM（Lidar Odometry and Mapping，Zhang 和 Singh，2014，细节未核对）的第一个想法：**只挑出局部"最尖"的点和"最平"的点**，分别叫边缘点（edge）和平面点（planar），用它们做匹配。

### 5.2 曲率定义

对同一条扫描线上的点 $p_i$，取其左右各若干个邻点组成集合 $S$（LOAM 中左右各 5 个，即 $|S|=10$，该数值未核对），定义平滑度（曲率）：

$$
c_i=\frac{1}{|S|\cdot\lVert p_i\rVert}\left\lVert\sum_{j\in S,\,j\ne i}\bigl(p_i-p_j\bigr)\right\rVert
$$

直觉：如果 $p_i$ 与邻点共线，向量差 $(p_i-p_j)$ 左右抵消，和接近 0，曲率小；如果 $p_i$ 是拐角，邻点都在同一侧，向量差累加，和很大。除以 $\lVert p_i\rVert$ 是为了让远处的点（点间距本来更大）不被误判，起到距离归一化的作用。

按 $c_i$ 排序：大的取为边缘点（如墙角、杆子），小的取为平面点（如地面、墙面）。为了特征分布均匀，会把一圈分成若干扇区，每个扇区各取固定数量；还要剔除"遮挡边缘"和"与激光束近乎平行的面"上的点，因为这些点的几何性质不稳定。

### 5.3 手算

示例（非项目实测）：2D 平面上一道 L 形墙，雷达在原点，扫描线上依次测得 9 个点（单位 m）：

$(0,5),(1,5),(2,5),(3,5),(4,5),(4,4),(4,3),(4,2),(4,1)$

编号 0 至 8，$(4,5)$ 为第 4 点（拐角）。取每侧 2 个邻点，$|S|=4$。

**第 4 点（拐角）**：邻点为 $(2,5),(3,5),(4,4),(4,3)$。

$$
\sum(p_4-p_j)=(2,0)+(1,0)+(0,1)+(0,2)=(3,3),\quad\lVert(3,3)\rVert=4.243
$$

$\lVert p_4\rVert=\sqrt{16+25}=6.403$。

$$
c_4=\frac{4.243}{4\times6.403}=0.1656
$$

**第 2 点（平直段）**：邻点 $(0,5),(1,5),(3,5),(4,5)$，差向量 $(2,0)+(1,0)+(-1,0)+(-2,0)=(0,0)$，$c_2=0$。

**第 3 点** $(3,5)$：邻点为 $(1,5),(2,5),(4,5),(4,4)$，差向量依次 $(2,0),(1,0),(-1,0),(-1,1)$，和为 $(1,1)$，范数 1.414；$\lVert p_3\rVert=5.831$，$c_3=1.414/(4\times5.831)=0.0606$。

**第 5 点** $(4,4)$：$c_5=0.0625$；**第 6 点** $(4,3)$：邻点全在竖直线上，$c_6=0$。

Python 复核结果：`c2=0, c3=0.0606, c4=0.1656, c5=0.0625, c6=0`。可见拐角点的曲率最大，约是相邻点的 2.7 倍；设阈值 0.1 就能只选出拐角。注意阈值要随传感器噪声和点间距调节，没有通用值。

> **注意**：噪声会让平直段的 $c$ 不为 0。真实系统是"取最大的若干个"而不是"超过某个固定阈值"，所以对噪声更稳健。

## 6. LOAM：点到线、点到面残差与一步高斯-牛顿（含手算）

### 6.1 残差定义

特征选出后，要把当前帧特征点 $\tilde p_i$（用当前位姿估计变换到地图系）与地图里的对应几何元素比较。

**边缘点—线**：在地图中找到距离 $\tilde p_i$ 最近的几个边缘点，拟合直线，取线上两点 $a,b$。点到直线距离：

$$
d_\varepsilon=\frac{\lVert(\tilde p_i-a)\times(\tilde p_i-b)\rVert}{\lVert a-b\rVert}
$$

分子是以 $(\tilde p_i-a)$ 与 $(\tilde p_i-b)$ 为邻边的平行四边形面积，除以底边 $\lVert a-b\rVert$ 就是高。

**平面点—面**：在地图平面点中找到三个近邻 $a,b,c$ 确定平面，法向 $\mathbf n=(b-a)\times(c-a)$。点到平面距离：

$$
d_\pi=\frac{\bigl|(\tilde p_i-a)\cdot\mathbf n\bigr|}{\lVert\mathbf n\rVert}
$$

总代价为所有残差平方和（带鲁棒核），用高斯-牛顿或列文伯格-马夸尔特法迭代求位姿增量。

### 6.2 手算：点到线、点到面

示例（非项目实测）。**点到线**（2D）：线上两点 $a=(1,0)$，$b=(1,2)$（竖直线 $x=1$），点 $p=(1.3,\,1.0)$。

$(p-a)=(0.3,1.0)$，$(p-b)=(0.3,-1.0)$。二维叉积 $u_xv_y-u_yv_x=0.3\times(-1.0)-1.0\times0.3=-0.6$，$\lVert a-b\rVert=2$，所以 $d_\varepsilon=0.6/2=0.3$ m，与"点离直线 $x=1$ 的水平距离 0.3"一致。

**点到面**（3D）：$a=(0,0,0),\ b=(2,0,0),\ c=(0,2,0.1)$，点 $q=(0.5,0.5,0.3)$。

$\mathbf n=(b-a)\times(c-a)=(2,0,0)\times(0,2,0.1)=(0\cdot0.1-0\cdot2,\;0\cdot0-2\cdot0.1,\;2\cdot2-0)=(0,-0.2,4)$，$\lVert\mathbf n\rVert=4.005$。

$(q-a)\cdot\mathbf n=0.5\times(-0.2)+0.3\times4=1.1$，所以 $d_\pi=1.1/4.005=0.2747$ m（Python 复核 `0.27466`）。

### 6.3 手算：一步高斯-牛顿（2D 点到线）

把问题缩小成 2D 并取平面点特征：两面墙 $x=2$（法向 $\mathbf n_1=(1,0)$）与 $y=3$（法向 $\mathbf n_2=(0,1)$）。当前帧特征点（已按初始位姿放入地图系）为

$$
(1.8,\,0.5),\ (1.8,\,-0.4)\ \text{对应墙 }x=2;\qquad(0.6,\,2.7),\ (-0.5,\,2.7)\ \text{对应墙 }y=3
$$

位姿增量 $\boldsymbol\xi=(\delta x,\delta y,\delta\theta)$，变换后 $q=R(\delta\theta)p+(\delta x,\delta y)$。残差 $r=\mathbf n^\top q-d_0$（$d_0$ 为墙的偏移）。一阶展开 $R(\delta\theta)p\approx p+\delta\theta\,Jp$，其中 $J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$，于是雅可比

$$
\frac{\partial r}{\partial\boldsymbol\xi}=\bigl[\,\mathbf n^\top,\ \ \mathbf n^\top Jp\,\bigr]
$$

初始残差：$r=(1.8-2,\,1.8-2,\,2.7-3,\,2.7-3)=(-0.2,-0.2,-0.3,-0.3)$。

四行雅可比：前两点 $\mathbf n=(1,0)$：$\mathbf n^\top Jp=-p_y$，得 $(1,0,-0.5)$ 与 $(1,0,0.4)$；后两点 $\mathbf n=(0,1)$：$\mathbf n^\top Jp=p_x$，得 $(0,1,0.6)$ 与 $(0,1,-0.5)$。

正规方程 $H\boldsymbol\xi=-g$，$H=\sum J^\top J$，$g=\sum J^\top r$：

$$
H=\begin{bmatrix}2&0&-0.1\\0&2&0.1\\-0.1&0.1&1.02\end{bmatrix},\qquad g=\begin{bmatrix}-0.4\\-0.6\\-0.01\end{bmatrix}
$$

解得 $\boldsymbol\xi=(0.2,\;0.3,\;\approx0)$。把它加回去后四个残差全部为 0（Python 复核 `r1=[0,0,0,0]`）。直觉很清楚：当前帧整体需要向 $+x$ 平移 0.2 m、向 $+y$ 平移 0.3 m 才能对齐两面墙。

> **提示**：注意 $H$ 的条件。如果只看到一面墙（只有 $\mathbf n=(1,0)$ 的点），$H$ 的第二行全零，沿墙方向的平移不可观，这就是长走廊退化的数学本质，详见第 14 节。

## 7. LOAM 的双频结构与后续改进

LOAM 把问题拆成两个并行线程：

- **里程计（高频，约 10 Hz）**：当前扫描对上一扫描的特征做匹配，求相邻帧运动，快但粗；
- **建图（低频，约 1 Hz）**：当前扫描对累积地图做匹配，慢但精，输出修正后的位姿。

两个频率的数值来自对论文的印象，未核对。核心思想是"用快的保证实时，用慢的保证精度"，这和 ORB-SLAM 的跟踪线程 + 局部建图线程同构。

LOAM 的问题与后续工作（只列名称，细节未核对）：

- **LeGO-LOAM（Shan 和 Englot，2018）**：先分割地面，减少无效点，针对嵌入式计算平台轻量化，并加入回环；
- **LIO-SAM（Shan 等，2020）**：引入 IMU 预积分与因子图，下一节展开；
- **FAST-LIO / FAST-LIO2（Xu 和 Zhang，2021–2022）**：用迭代误差状态卡尔曼滤波做紧耦合，增量式 ikd-tree 地图，未核对细节。

## 8. LIO-SAM：因子图与紧耦合

### 8.1 为什么要"因子图"

[SLAM 问题](./01_SLAM问题_前端后端_位姿图优化.md)一章讲过：把待估计量（位姿、偏置）作为变量节点，把每一条测量约束作为因子节点，整张图的最大后验估计等价于一个非线性最小二乘。LIO-SAM（Shan 等，2020）的贡献是：**把 IMU、激光里程计、GPS、回环四种信息都做成因子**，放进同一张图，由增量求解器 iSAM2 在线更新（论文细节未核对）。

### 8.2 四种因子

| 因子 | 连接的变量 | 约束内容 |
|---|---|---|
| IMU 预积分因子 | 相邻关键帧的位姿、速度、IMU 偏置 | 两关键帧之间的相对运动应与 IMU 积分一致 |
| 激光里程计因子 | 相邻关键帧位姿 | 由特征点对局部地图配准得到的相对位姿 |
| GPS 因子 | 某关键帧位姿 | 绝对位置约束，提供全局锚点，可选 |
| 回环因子 | 两个远隔的关键帧位姿 | 回环检测后 ICP 求得的相对位姿 |

IMU 预积分把两关键帧之间成百上千个 IMU 采样"压缩"成一个相对位姿/速度的增量，使优化时不必重新积分。

### 8.3 关键帧与滑窗

并不是每帧激光都进图。当位姿变化超过一定阈值（距离或角度，具体值依配置）才新建关键帧。局部匹配不对全局地图，而是把最近若干关键帧的特征点拼成一个**滑窗局部地图**，既保证效率，又让匹配参考相对准确。

### 8.4 用 IMU 去畸变

LIO-SAM 把预积分得到的扫描内姿态用于点云去畸变（第 4 节的 $\theta_i,\mathbf t_i$ 由 IMU 积分提供，比匀速模型准），再把去畸变后的位姿作为激光匹配的初值，匹配结果反过来约束 IMU 偏置。这种"两个传感器互相校正"就是紧耦合的含义。

> **面试要点**：紧耦合（tightly coupled）= IMU 与激光的原始测量共同进入同一个优化/滤波；松耦合（loosely coupled）= 先各自算出位姿，再融合位姿结果。紧耦合在激光退化时能借助 IMU 维持估计，松耦合则不能。

## 9. gmapping：Rao-Blackwellized 粒子滤波

gmapping（Grisetti、Stachniss、Burgard，2007，细节未核对）是 2D 激光 SLAM 最经典的开源实现，思想是 Rao-Blackwellization：把联合分布 $p(x_{1:t},m\mid z_{1:t},u_{1:t})$ 分解为

$$
p(x_{1:t},m\mid z_{1:t},u_{1:t})=p(m\mid x_{1:t},z_{1:t})\;p(x_{1:t}\mid z_{1:t},u_{1:t})
$$

用粒子滤波估计轨迹 $x_{1:t}$，每个粒子带一张自己的栅格地图 $m$（已知轨迹时建图是解析的）。核心步骤：

1. 用里程计提议分布采样新位姿，并用扫描匹配做"精修提议"（gmapping 的改进之处）；
2. 用扫描与该粒子地图的匹配度更新权重；
3. 当有效粒子数 $N_{\text{eff}}=1/\sum_i w_i^2$ 低于阈值（常取 $N/2$）时重采样；
4. 每个粒子各自更新自己的地图。

示例（非项目实测）：5 个粒子的归一化权重为 $(0.5,0.3,0.1,0.05,0.05)$。

$$
\sum w_i^2=0.25+0.09+0.01+0.0025+0.0025=0.355,\qquad N_{\text{eff}}=1/0.355=2.82
$$

Python 复核：`2.8169`。$2.82<5/2$，触发重采样。

优点是实现简单、2D 室内小场景效果不错；缺点是每粒子一张地图，**内存和计算与粒子数成正比**，场景一大就吃不消，且重采样可能造成粒子贫化（particle depletion），丢掉真正的回环假设。所以大场景倾向图优化方案。

## 10. Cartographer：子图与分支定界（含手算）

### 10.1 总体思路

Cartographer（Hess、Kohler、Rapp、Andor，2016，细节未核对）面向"实时、2D 大场景、带回环"。它把地图切成许多**子图（submap）**，每个子图只由局部连续的若干扫描构成，子图内部的漂移很小可以忽略；子图之间的相对位姿靠后端优化。

- **局部 SLAM（前端）**：每个新扫描（先经 IMU/里程计外推，再体素滤波）用相关性扫描匹配（Ceres 实现）对齐到当前活动子图，插入后更新子图的概率栅格；
- **全局 SLAM（后端）**：子图完成后，回环检测对每个新扫描在所有已完成子图中搜索匹配，成功则加入稀疏位姿调整（SPA）约束，周期性地全局优化。

### 10.2 概率栅格更新

每个栅格存占据概率。命中（hit）时概率向上更新，穿过（miss）时向下更新，用对数几率（odds）形式相乘：

$$
\text{odds}(p)=\frac{p}{1-p},\qquad \text{odds}(p_{\text{new}})=\text{odds}(p_{\text{old}})\cdot\text{odds}(p_{\text{hit}})
$$

示例（非项目实测）：初始 $p=0.5$，取 $p_{\text{hit}}=0.55$。第 1 次命中 $p=0.55$；第 2 次 $\text{odds}=(0.55/0.45)^2=1.494$，$p=0.599$；第 3 次 $p=0.646$（Python 复核：`0.55, 0.599, 0.646`）。多次观测后概率收敛，单次噪声被抑制。

### 10.3 分支定界（Branch and Bound）

回环检测要在大范围内搜索"扫描相对子图的位姿 $(x,y,\theta)$"。暴力搜索是三维的、范围大、分辨率细，太慢。分支定界的做法：

1. 把搜索空间按平移分块；每个**节点**代表一块候选位姿；
2. 对每个节点计算一个**上界**：它覆盖的所有细粒度位姿的最大可能得分；
3. 用**深度优先**搜索，先扩展上界最高的节点；一旦在叶子（最细分辨率）找到一个得分，就把它作为当前最优；之后上界低于当前最优的节点整体被**剪枝**。

上界的计算用"预先计算的粗分辨率栅格"：粗栅格的每个格子存储其覆盖的细栅格中的最大概率值，所以粗节点得分总是不小于它下面任何细节点的得分。

手算示例（非项目实测）：细栅格 $4\times4$ 的概率为

$$
\begin{bmatrix}0.1&0.2&0.1&0.0\\0.2&0.9&0.8&0.1\\0.1&0.7&0.9&0.2\\0.0&0.1&0.2&0.1\end{bmatrix}
$$

构造 $2\times2$ 粗栅格：每个格子取其覆盖的 $2\times2$ 细格的最大值，得 $(0.9,\,0.8;\ 0.7,\,0.9)$（Python 复核）。若一次叶子搜索已找到得分 0.8 的候选，则任何上界小于 0.8 的粗节点（如其他分支上界 0.7）可直接丢弃。这就是"只展开可能更好的分支"，让搜索从指数降下来。

> **注意**：上界必须是真正的上界，否则会剪错分支；这也是为什么粗栅格取"最大"而非平均。

## 11. slam_toolbox：Karto 图优化与终身建图

slam_toolbox（Macenski 和 Jambrecina，2021，细节未核对）是 ROS2 Humble 上最常用的 2D 激光 SLAM。其特点：

- 基于 Karto（OpenKarto）的扫描匹配与稀疏位姿图，后端用 Ceres 求解；
- 提供在线同步/异步模式、**终身建图**（lifelong mapping）模式：可加载旧地图后继续在里面建图、增删节点；
- 提供序列化地图（保存位姿图本身，而不只是栅格图），可以在加载后做图的编辑；
- 与 Nav2 通过 `map` 话题和 `map -> odom` 的 TF 衔接。

这里 `map->odom` 的发布者在建图时是 SLAM 节点，在已有地图上导航时通常是 AMCL（见 [粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)）。两者在同一时刻只能有一个负责这个变换，否则 TF 会冲突。

## 12. 系统对比表

| 项目 | 维度 | 滤波/图 | 特点 | 适用场景 | 局限 |
|---|---|---|---|---|---|
| gmapping | 2D | 粒子滤波 | 实现简单、依赖里程计质量 | 小型室内、里程计好 | 大场景内存爆、无显式回环 |
| Cartographer | 2D/3D | 图（子图 + SPA） | 实时回环、效果好、可用 IMU | 中大型场景 | 配置多、参数调优复杂 |
| slam_toolbox | 2D | 图（Karto） | ROS2 友好、终身建图 | Nav2 配套 | 超大场景性能受限 |
| LOAM | 3D | 特征 + 滑动优化 | 高精度里程计 | 室外 3D | 无 IMU 紧耦合、无回环 |
| LIO-SAM | 3D | 因子图 | IMU 紧耦合 + 回环 + GPS | 车载/手持 3D | 需 9 轴 IMU，参数多 |

> **提示**：面试被问"你会选哪个"时，不要只报名字，要给场景：小车在室内、Nav2 配套，选 slam_toolbox；需要多层楼与 IMU，选 Cartographer；3D 激光加 IMU，选 LIO-SAM 或类似紧耦合方案。

## 13. 在本项目里怎么用

七个项目的 CV 里没有做激光 SLAM 的内容，本章属于**延伸知识**，不是项目内容，面试时不要把自己说成实现过 LOAM 或 Cartographer。可以诚实使用的关联：

- **P5 车式移动机器人导航**：P5 使用 2D 激光做地图内的定位（EKF/AMCL），地图本身可来自上述任一种建图方法。AMCL 的"扫描与栅格地图匹配"和 gmapping/Cartographer 的前端匹配是同一类思想；EKF 融合 IMU/编码器/激光的做法，对应 LIO 的"紧耦合"思路的简化版。见 [P5 EKF 融合定位](../../01_项目课程/P5_车式移动机器人自主导航/04_EKF融合定位.md) 与 [AMCL 粒子滤波定位](../../01_项目课程/P5_车式移动机器人自主导航/05_AMCL粒子滤波定位.md)。P5 遇到的"漂移与位姿跳变"（时间、坐标系、运动方向不一致）在激光 SLAM 里同样是头号故障源，参见 [传感器与数据质量](../../01_项目课程/P5_车式移动机器人自主导航/03_传感器与数据质量_时间坐标与运动方向.md)。
- **P6 M-Robots 平台**：多源传感器的时间戳、坐标系和有效性统一，是任何 SLAM 前端的前提，参见 [机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)。
- **P2 / P1**：不直接使用激光 SLAM。

被问到"你做过 SLAM 吗"时的诚实表述：我的导航项目使用已建好的栅格地图做定位，没有自己实现建图；但我理解建图的前端匹配和后端优化，并能解释它们与我用的 EKF/AMCL 的关系。

## 14. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 地图中墙壁"重影"、变弯 | 未去畸变，或雷达与里程计时间戳未对齐 | 检查扫描时间戳和 TF 查找时间；加去畸变 |
| 长走廊中位姿沿走廊"滑动" | 几何退化：只有两面平行墙，沿走廊方向不可观（$H$ 秩亏） | 加 IMU/里程计约束，检测退化方向后降权 |
| 回环后地图撕裂或扭曲 | 误回环（相似场景），未做几何验证 | 提高回环阈值，加鲁棒核，做一致性检验 |
| 地图越走越"歪" | IMU 外参错，或 LiDAR 与 IMU 的坐标轴定义不一致 | 重新标定外参，核对坐标系右手系 |
| CPU 满载，地图延迟 | 点云太密、体素太小、粒子过多 | 体素下采样，降低发布频率，减小粒子数 |
| 动态物体（人）在地图中留痕 | 把移动物体当静态观测累积 | 利用概率栅格 miss 更新，或动态物体过滤 |
| gmapping 在大场景崩溃 | 每粒子一张地图，内存随粒子数线性增长 | 换用图优化方案，或减小粒子数/地图分辨率 |
| `map->odom` 跳变 | 回环优化修正了整条轨迹 | 这是正常现象，局部规划应使用 `odom` 系保持连续 |

最后一行值得多说一句：SLAM 后端改轨迹是**离散跳变**，所以导航栈里把"连续但会漂"的 `odom` 系和"全局一致但会跳"的 `map` 系分开，P5 的 TF 分工就是这个道理。

## 15. 面试怎么问、怎么答

**Q1. How does LOAM choose features, and why?**

> LOAM computes a smoothness value for each point from the sum of differences to its neighbours on the same scan line. Points with large values are edge points, and points with small values are planar points. The reason is efficiency: instead of registering tens of thousands of raw points, I only match a few hundred distinctive ones using point-to-line and point-to-plane distances.

中文：LOAM 用同一扫描线上邻点差向量之和算曲率，大的当边缘点，小的当平面点；这样只需匹配几百个有区分度的点，用点到线、点到面距离做残差，比逐点 ICP 快得多。追问方向：为什么除以点的距离？（距离归一化，避免远处点间距大被误判。）

**Q2. What is motion distortion and how do you correct it?**

> A spinning LiDAR takes about a tenth of a second per revolution, so if the robot moves during that time, points measured at different moments are in different poses. I correct it by assigning each point a timestamp within the scan, interpolating the pose from a constant-velocity model or IMU integration, and transforming every point into a common frame, usually the scan start.

中文：旋转雷达一圈有时间跨度，期间机器人运动会让点云变形；给每个点时间戳，用匀速模型或 IMU 积分插值位姿，把所有点变换到同一参考系（常取扫描起点）。追问：角速度和线速度哪个影响更大？（角速度，因为误差随距离放大。）

**Q3. What is the difference between filter-based and graph-based SLAM, and where do gmapping and Cartographer fit?**

> Filter-based SLAM, like gmapping, maintains a belief over the current state and map and marginalises out the past, so it cannot easily revise old estimates. Graph-based SLAM, like Cartographer or slam_toolbox, keeps the trajectory as a pose graph and re-optimises all poses when a loop closure arrives, which scales better to large maps and fixes accumulated drift.

中文：滤波式只保留当前信念，过去被边缘化，难以回头修正；图式保留整条轨迹，回环来了全局重优化，扩展性更好、能消除累计漂移。追问：为什么 gmapping 粒子数多了内存会爆？（每粒子一张地图。）

**Q4. In LIO-SAM, what factors are in the graph?**

> There are IMU preintegration factors between keyframes, LiDAR odometry factors from scan-to-local-map matching, optional GPS factors for absolute position, and loop closure factors from ICP between distant keyframes. A solver called iSAM2 updates the graph incrementally. This is tightly coupled because the IMU and LiDAR constrain the same variables.

中文：四类因子：IMU 预积分、激光里程计、可选 GPS、回环；iSAM2 增量求解；因为两个传感器约束同一组变量，所以是紧耦合。追问：为什么要用关键帧而不是每帧入图？（控制图规模、避免冗余约束。）

**Q5. Why does Cartographer use branch and bound?**

> Loop closure needs to search a large space of poses. Branch and bound computes an upper bound on the score for each group of candidate poses using a coarse precomputed grid, and prunes any branch whose bound is lower than the best score found so far. That keeps the search tractable and still guarantees the global optimum for the matching score.

中文：回环要在大位姿空间里搜索；分支定界用粗分辨率栅格给一组候选位姿算得分上界，上界不如当前最优的整支剪掉，既快又保证匹配得分全局最优。

## 16. 本章要点速记

1. 激光 SLAM 统一结构：去畸变、特征/栅格、匹配、关键帧/子图、回环、后端优化。
2. 运动畸变由扫描期间的运动引起，角速度影响随距离放大，必须用时间戳插值去畸变。
3. LOAM 曲率 $c_i=\frac{1}{|S|\lVert p_i\rVert}\lVert\sum(p_i-p_j)\rVert$，大的是边缘点，小的是平面点。
4. 点到线用叉积除底边，点到面用法向投影；残差平方和用高斯-牛顿求解。
5. 只看到一面墙或两面平行墙时，$H$ 秩亏，沿墙方向不可观（几何退化）。
6. LIO-SAM = 因子图（IMU 预积分 + 激光里程计 + GPS + 回环）+ iSAM2 增量求解。
7. gmapping 用 RBPF，每粒子一张地图，$N_{\text{eff}}=1/\sum w_i^2$ 触发重采样。
8. Cartographer 用子图 + 分支定界回环 + SPA 后端；粗栅格取最大值保证上界。
9. slam_toolbox 是 ROS2 配套的 2D 图优化方案，支持终身建图与序列化地图。
10. 本人项目属于"使用地图做定位与导航"，不是实现建图；回答要诚实。

## 17. 自测题

1. 为什么曲率要除以 $\lVert p_i\rVert$？（距离归一化，远处点间距更大，否则平面上的远点会被误判为尖锐。）
2. $v=2$ m/s、$\omega=0.2$ rad/s、$T=0.1$ s，点在 $s=1$ 时刻，距离 10 m；仅考虑转动，点位偏差约多少？（$\theta=0.02$ rad，偏差约 $10\times0.02=0.2$ m。）
3. 只看到一面墙时哪个方向不可观？（沿墙方向的平移，因为法向量没有该方向的分量。）
4. 粒子权重 $(0.4,0.4,0.2)$，$N_{\text{eff}}$ 是多少？（$1/(0.16+0.16+0.04)=2.78$。）
5. 为什么分支定界的粗栅格取子块的最大值而不是平均？（必须是上界，才能安全剪枝而不丢掉最优解。）

## 18. 相关链接

- 上一章：[视觉里程计：特征、对极几何、PnP 与 BA](./02_视觉里程计_特征_对极几何_PnP_BA.md)
- 下一章：[视觉惯性里程计 VIO：预积分与滑动窗口](./04_视觉惯性里程计VIO_预积分_滑动窗口.md)
- 基础：[点云处理：滤波、配准与 ICP](../09_感知与视觉/02_点云处理_滤波_配准_ICP.md)、[粒子滤波与 AMCL](../06_状态估计/05_粒子滤波与AMCL.md)
- 项目：[P5 车式移动机器人导航 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[07_SLAM与建图 README](./README.md)
