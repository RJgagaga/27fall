# 代价地图、局部规划与 TEB

> **一句话定位**：全局路径只考虑静态地图；车一动，周围障碍就会变。代价地图把传感器看到的障碍变成"每个格子多危险"的数字，局部规划器（DWA、TEB）在这张图上、沿着全局路径，实时优化出车能走、也躲得开的短轨迹。本章对应 P5 CV 中 "TEB local trajectory optimisation"。

## 本章学完你能……

- 说明占据栅格怎样用对数几率融合多次观测，并手算一次更新。
- 解释代价地图的数值含义（0 / 253 / 254 / 255）、膨胀公式，手算不同距离处的膨胀代价，说清各层（static / obstacle / inflation）的分工。
- 简述 DWA 的"速度窗口 + 轨迹模拟 + 打分"，并说出它为何不适合车式底盘。
- 说明 TEB（Timed Elastic Band）的构造：位姿序列 + 时间间隔序列；写出各代价项（路径跟随、障碍、速度、加速度、非完整运动学、最小转弯半径、时间最优），并理解它们为何以"罚函数"形式出现。
- 理解 g2o 超图：顶点、边（含超边）、稀疏性，以及内 / 外迭代、自动调整带长、同伦类并行规划。
- 手算一个三位姿带的代价曲线，得到解析最优 $y=0.5$，并体会软约束的含义。
- 用英文回答 TEB / 代价地图 / 局部规划的面试问题。

## 前置知识

- 全局规划：[图搜索](./01_图搜索_Dijkstra_A星.md)、[Hybrid A*](./02_Hybrid_A星与车辆运动学约束.md)
- 最优化基础（非线性最小二乘、稀疏矩阵）：[最优化与二次规划](../01_数学/04_最优化与二次规划.md)
- 自行车模型：[差速、麦克纳姆、全向轮与车辆运动学](../02_力学与建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)

## 目录

1. 占据栅格
2. 代价地图与膨胀
3. 代价地图的分层结构
4. 局部规划的任务与 DWA 简述
5. TEB：构造
6. TEB：代价项
7. TEB：优化与 g2o 超图
8. 手算：三位姿带的优化
9. 从轨迹到控制指令；重规划
10. 在本项目里怎么用
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记、自测题、相关链接

---

## 1. 占据栅格

### 1.1 它要解决什么问题

激光每帧只看到一部分环境，而且有噪声。我们要用多帧数据判断"这个格子到底有没有障碍"，并给出一个**概率**。

### 1.2 对数几率更新

设格子被占据的概率 $p$，定义**对数几率**（log-odds）：

$$
l=\ln\frac p{1-p},\qquad p=\frac1{1+e^{-l}}
$$

用对数几率的好处：贝叶斯更新变成**加法**。每次观测只加一个常数：

$$
l_t=l_{t-1}+l_{sensor}-l_0
$$

其中 $l_0=\ln\frac{p_0}{1-p_0}$ 是先验（先验 0.5 时 $l_0=0$）。激光光束终点所在格子给"命中"增量 $l_{occ}$，光束穿过的格子给"空闲"增量 $l_{free}$（负值）。

### 1.3 手算

**示例**（教学数值）：$l_{occ}=+0.85$（对应 $p\approx0.70$），$l_{free}=-0.40$（对应 $p\approx0.40$），先验 $l_0=0$，初始 $l=0$（$p=0.5$）。

- 第 1 次命中：$l=0.85$，$p=1/(1+e^{-0.85})=0.700$。
- 第 2 次命中：$l=1.70$，$p=1/(1+e^{-1.70})=1/(1+0.1827)=0.8455$。
- 第 3 次观测为"空闲"：$l=1.70-0.40=1.30$，$p=1/(1+e^{-1.30})=1/(1+0.2725)=0.7858$。

观察：多次一致的观测使概率越来越大；偶尔的相反观测只拉低一点点，所以单帧噪声不会推翻已确立的障碍。通常设置上下限（$l\in[l_{min},l_{max}]$）避免饱和后"翻不过来"（动态障碍离开后要能被清除）。

### 1.4 分类

概率设阈值转成三类：**占据**（$p>0.65$ 左右）、**空闲**（$p<0.2$ 左右）、**未知**。导航里用到的代价地图再把它转成离散的代价值（见下节）。阈值数值因软件而异。

---

## 2. 代价地图与膨胀

### 2.1 它要解决什么问题

规划器想知道的不是"有没有障碍"，而是**"车站在这个格子里会不会碰、危险程度多大"**。机器人有体积，所以障碍周围一圈也要标成危险。这就是**膨胀**（inflation）。

### 2.2 代价值约定（ROS costmap_2d）

代价地图每格一个 $0\sim255$ 的无符号字节：

| 值 | 名称 | 含义 |
|---|---|---|
| 0 | FREE_SPACE | 自由 |
| 1~252 | 递减代价 | 离障碍越近越大，可通行但不推荐 |
| 253 | INSCRIBED_INFLATED_OBSTACLE | 机器人**中心**在这里，**内切圆**会碰到障碍，任意朝向都碰 |
| 254 | LETHAL_OBSTACLE | 格子本身是障碍 |
| 255 | NO_INFORMATION | 未知 |

（数值约定凭记忆，以所用版本文档为准。）

### 2.3 膨胀公式

设某格到最近障碍的距离为 $d$，机器人内切圆半径 $r_{ins}$，`cost_scaling_factor` 为 $k$：

$$
\text{cost}(d)=\begin{cases}254&d=0\\253&0<d\le r_{ins}\\252\cdot e^{-k\,(d-r_{ins})}&d>r_{ins}\ (\text{直到 inflation\_radius})\end{cases}
$$

超过 `inflation_radius` 后为 0。$k$ 越大，代价衰减越快（障碍的"影响圈"越窄）。

### 2.4 手算

**示例**：$r_{ins}=0.30$ m，$k=3.0$。

| $d$ (m) | 计算 | 代价 |
|---|---|---|
| 0.2 | $\le r_{ins}$ | 253 |
| 0.4 | $252e^{-3(0.4-0.3)}=252e^{-0.3}$ | 186.7 → 186 |
| 0.5 | $252e^{-0.6}$ | 138.3 → 138 |
| 0.7 | $252e^{-1.2}$ | 75.9 → 75 |
| 1.0 | $252e^{-2.1}$ | 30.9 → 30 |

规划器沿代价低的格子走，路径自然离障碍远一些，但并不是"硬禁止"。$k$ 减半为 1.5：$d=0.5$ 的代价是 $252e^{-0.3}=186.7$——障碍影响范围变宽，路径会更远离障碍，狭窄通道里也更容易"被封死"。

### 2.5 footprint：内切圆与外接圆

- **内切圆半径** $r_{ins}$：车体内能放下的最大圆。中心在距障碍 $\le r_{ins}$ 的格子里，则无论朝向都会碰。
- **外接圆半径** $r_{circ}$：能包住整个车体的最小圆。中心距障碍 $>r_{circ}$ 才保证无碰撞。

对长方形车，两个半径差别大（例如长 0.7 宽 0.4：$r_{ins}=0.2$，$r_{circ}=\sqrt{0.35^2+0.2^2}=0.40$，以车体中心为圆心），这时只用圆近似会过于保守或不安全，需要用多边形 footprint 或多圆（见 [Hybrid A* 章](./02_Hybrid_A星与车辆运动学约束.md) 第 8 节）。

---

## 3. 代价地图的分层结构

costmap_2d 由多个**层**（layer）叠加，每层往"主地图"（master grid）里写入自己的代价，最后取最大值：

| 层 | 内容 | 来源 |
|---|---|---|
| Static layer | 已知静态地图 | `/map`（地图服务器） |
| Obstacle layer / Voxel layer | 实时传感器障碍；含射线追踪清除 | LiDAR 点云 / 激光 |
| Inflation layer | 对前面所有障碍做膨胀 | 参数：`inflation_radius`、`cost_scaling_factor` |
| 其他（如 social layer） | 行人区域等 | 可选 |

**全局代价地图**（global costmap）：整张地图；用静态层 + 障碍层 + 膨胀，更新慢；给全局规划器用。

**局部代价地图**（local costmap）：以机器人为中心的**滑动窗口**（rolling window，如 $5\times5$ m），更新快（数 Hz 到十几 Hz），只关心附近的动态障碍；给局部规划器用。

**障碍层的"标记"与"清除"**：命中点标记为障碍；光束经过的格子通过**射线追踪**清除。动态障碍离开后才会消失；有玻璃、低矮障碍时雷达看不见，代价地图里就没有，是常见"撞"的原因。

> 其他重要参数：`update_frequency`、`publish_frequency`、`footprint`（或 `robot_radius`）、`obstacle_range`（障碍标记范围）、`raytrace_range`（清除范围）。

---

## 4. 局部规划的任务与 DWA 简述

### 4.1 任务

全局路径是静态的、粗的；局部规划器在局部代价地图上，每个控制周期（常 10~20 Hz）算出**短期内**的运动，要求：

1. 大致沿全局路径；
2. 避开实时障碍；
3. 满足速度 / 加速度 / 运动学限制；
4. 输出控制指令（$v,\omega$ 或转向角）。

### 4.2 DWA（Dynamic Window Approach）

DWA 的流程：

1. **速度窗口**：当前速度 $(v_c,\omega_c)$，最大加速度 $a_v,a_\omega$，控制周期 $\Delta t$。一个周期内能达到的速度范围：

$$
v\in[v_c-a_v\Delta t,\ v_c+a_v\Delta t],\qquad \omega\in[\omega_c-a_\omega\Delta t,\ \omega_c+a_\omega\Delta t]
$$

2. **采样**窗口内的 $(v,\omega)$ 对；
3. 对每个采样，假设速度恒定，向前**模拟**一段时间（如 1.5~3 s）得到圆弧轨迹；
4. 去掉会碰撞的、或不能在碰撞前停下来的；
5. 对剩余轨迹**打分**：

$$
G(v,\omega)=\alpha\cdot\text{heading}+\beta\cdot\text{clearance}+\gamma\cdot\text{velocity}
$$

   分别衡量朝目标 / 全局路径、离障碍距离、速度大小；
6. 选得分最高者执行。

**手算窗口**：当前 $v_c=0.5$ m/s，$a_v=1.0$ m/s²，$\Delta t=0.2$ s：$v\in[0.3,0.7]$。

### 4.3 DWA 的局限（尤其对车式底盘）

- 假设可以直接选取 $(v,\omega)$ 独立变化，对差速底盘合理；**车式底盘的 $\omega$ 与 $v$ 通过转向角耦合**（$\omega=v\tan\delta/L$），$v\approx0$ 时无法转向，且转向角有速率限制，采样出的很多轨迹不可行；
- 每个周期都重新采样，没有时间连续性，轨迹可能抖动；
- 不考虑**时间**的优化，只是给定恒速轨迹打分；
- 容易陷入局部极小（如凹形障碍前）。

这就是为什么车式机器人常选 TEB（显式包含非完整约束与最小转弯半径）、MPPI 或 RPP 等。

---

## 5. TEB：构造

### 5.1 它要解决什么问题

我们想要一条轨迹，不仅"路径形状"好，而且**带时间**——每一段花多长时间，从而速度、加速度都能计算和约束，并且总时间尽量短。

> 原始文献：C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann, T. Bertram，"Trajectory modification considering dynamic constraints of autonomous robots"（ROBOTIK 2012）；"Efficient trajectory optimization using a sparse model"（ECMR 2013）。"Elastic band"思想来自 Quinlan 与 Khatib（1993）。ROS 包 `teb_local_planner`。（凭记忆引用，未核对原文。）

### 5.2 弹性带（elastic band）的直觉

想象一根橡皮筋，两端钉在起点和目标，中间挂在全局路径附近。

- 有"**内力**"：让橡皮筋收紧、变短；
- 有"**外力**"：被障碍推开；
- 受到约束：形状必须满足车辆的运动能力。

TEB 在"橡皮筋"（elastic band）上又加了**时间**（timed）维度。

### 5.3 变量

TEB 由位姿序列和时间间隔序列组成：

$$
B=\{\,s_0,\ \Delta T_0,\ s_1,\ \Delta T_1,\ \dots,\ \Delta T_{n-1},\ s_n\,\}
$$

| 符号 | 含义 | 单位 |
|---|---|---|
| $s_i=(x_i,y_i,\beta_i)$ | 第 $i$ 个位姿（位置 + 朝向） | m, m, rad |
| $\Delta T_i$ | 从 $s_i$ 到 $s_{i+1}$ 所需时间 | s |
| $n+1$ | 位姿个数 | |

$s_0$（当前位姿）和 $s_n$（局部目标）固定，中间 $s_1\ldots s_{n-1}$ 与全部 $\Delta T_i$ 是**优化变量**。

由这些变量可以直接算出：

- 速度：$v_i=\dfrac{\|p_{i+1}-p_i\|}{\Delta T_i}$，角速度 $\omega_i=\dfrac{\beta_{i+1}-\beta_i}{\Delta T_i}$（朝向差归一化到 $(-\pi,\pi]$）；
- 加速度：$a_i=\dfrac{2\,(v_{i+1}-v_i)}{\Delta T_i+\Delta T_{i+1}}$。

### 5.4 优化问题

$$
B^*=\arg\min_B\ \sum_k\gamma_k\,f_k(B)
$$

每个 $f_k$ 是一个代价项，$\gamma_k$ 是权重。**约束被写成罚函数**（超出范围才有代价），所以这是一个**无约束的非线性最小二乘**问题。

---

## 6. TEB：代价项

罚函数的一般形式（teb 里的做法）：对变量 $x$ 与上限 $a$、容差 $\epsilon$：

$$
e_{upper}(x,a,\epsilon)=\max\big(0,\ x-(a-\epsilon)\big),\qquad e_{lower}(x,a,\epsilon)=\max\big(0,\ (a+\epsilon)-x\big)
$$

代价取 $e^2$。$\epsilon$ 提前一点开始惩罚，作为"安全边距"。

| 代价项 | 作用 | 形式（简化） |
|---|---|---|
| **路径跟随 / via-point** | 让轨迹贴近全局路径（或经过某些点） | $\|p_i-p_{ref}\|^2$ |
| **障碍** | 离障碍距离小于 $d_{min}$ 就惩罚 | $e_{lower}(d_i,\,d_{min},\,\epsilon)$ |
| **速度** | 线速度 / 角速度不超过限制（含倒车限制） | $e_{upper}(v_i,\,v_{max},\,\epsilon)$ |
| **加速度** | 线 / 角加速度不超过限制 | $e_{upper}(|a_i|,\,a_{max},\,\epsilon)$ |
| **非完整运动学** | 相邻位姿近似位于同一圆弧上（车不能侧移） | 见下 |
| **最小转弯半径** | 曲率不超过 $1/R_{min}$ | $e_{upper}(|\omega_i|/|v_i|,\ 1/R_{min},\ \epsilon)$ |
| **时间最优** | 总时间尽量短 | $(\sum\Delta T_i)^2$ 或 $\Delta T_i^2$ |
| **最短路径** | 总长度尽量短 | $\|p_{i+1}-p_i\|^2$ |
| **偏好转向方向 / 前进** | 避免来回切换，偏好前进 | 附加项 |

### 6.1 非完整运动学项的推导

车（后轴中心）的速度沿车身方向。相邻位姿 $s_i,s_{i+1}$，位移 $d=p_{i+1}-p_i$，航向单位向量 $h_i=(\cos\beta_i,\sin\beta_i)$。如果两点在**同一圆弧**上，则"航向 $h_i$ 与弦 $d$ 的夹角"等于"弦 $d$ 与航向 $h_{i+1}$ 的夹角"（弦切角相等）。用二维叉积 $a\times b=a_xb_y-a_yb_x$ 表示：

$$
h_i\times d=d\times h_{i+1}\ \Longleftrightarrow\ (h_i+h_{i+1})\times d=0
$$

展开得

$$
e_{nh}=(\cos\beta_i+\cos\beta_{i+1})\,\Delta y-(\sin\beta_i+\sin\beta_{i+1})\,\Delta x
$$

$e_{nh}=0$ 时两位姿位于同一圆弧（含直线）。把 $e_{nh}^2$ 作为惩罚项：越偏离，越"像侧移"，代价越大。

**手算**：$s_i=(0,0,0)$，$s_{i+1}=(1,0,0.2)$。$\Delta x=1,\Delta y=0$。$e_{nh}=(1+\cos0.2)\cdot0-(0+\sin0.2)\cdot1=-0.1987$。非零：这对位姿**不在同一圆弧上**（从 $(0,0)$ 沿 0° 直行到 $(1,0)$ 后车头却变成 0.2 rad，需要原地转，车做不到）；若改成 $s_{i+1}=(0.995,\ 0.0998,\ 0.2)$（沿圆弧到达，$R=5$），则 $\Delta y=0.0998$，$e_{nh}=(1+0.980)(0.0998)-(0.1987)(0.995)=0.1976-0.1977\approx0$ ✓。

### 6.2 时间最优项

$\Delta T_i$ 越小越快，所以惩罚 $\Delta T$ 本身，使轨迹**尽可能快**；但速度 / 加速度项又限制快的程度，二者的平衡得到"在限制内尽量快"。这就是 TEB 的"时间最优"。

> **注意**：时间最优不等于"一味冲"。障碍项、加速度项、运动学项会把速度压下来；权重决定谁优先。

---

## 7. TEB：优化与 g2o 超图

### 7.1 超图

g2o（General Graph Optimization）把问题表示成**图**：

- **顶点**（vertex）：优化变量——每个位姿 $s_i$ 是一个顶点，每个 $\Delta T_i$ 是一个顶点；
- **边**（edge）：一个代价项，连接它依赖的变量。

某些代价项依赖多个变量，所以是**超边**（hyper-edge）：

| 边 | 连接的顶点 |
|---|---|
| 障碍 / via-point | 1 个位姿（一元） |
| 非完整运动学 | 2 个位姿 |
| 速度 | 2 个位姿 + 1 个 $\Delta T$ |
| 加速度 | 3 个位姿 + 2 个 $\Delta T$ |
| 时间最优 | 1 个 $\Delta T$ |

```text
 s0 ─ΔT0─ s1 ─ΔT1─ s2 ─ΔT2─ s3 ...
 │         │         │         │
 障碍边    障碍边    障碍边   (一元边)
 └──速度边(s0,s1,ΔT0)  └──速度边(s1,s2,ΔT1) ...
      └─────加速度边(s0,s1,s2,ΔT0,ΔT1)──────┘ ...
```

### 7.2 稀疏性

每个代价项只涉及相邻几个变量，所以 Gauss-Newton / Levenberg-Marquardt 的 Hessian 近似 $H=J^\top\Omega J$ 是**带状稀疏**矩阵。用稀疏 Cholesky 求解，复杂度近似线性于位姿个数，才能在每个控制周期（十几 Hz）内完成。

### 7.3 内外迭代

teb 的参数有"内迭代 / 外迭代"：

- **外迭代**：每轮先**自动调整带长**（auto-resize）——若某段 $\Delta T_i$ 超过参考时间间隔 $dt_{ref}$ 加滞回 $dt_{hyst}$ 就插入位姿，太短就删掉，使位姿间隔近似均匀，再建图；
- **内迭代**：对当前图做若干步优化。

### 7.4 同伦类（homotopy class）并行规划

障碍两侧各绕一次是两种**拓扑不同**的路径（同伦类不同）。单个局部优化器只能收敛到初值所在的局部极小。`HomotopyClassPlanner` 同时生成多条不同拓扑的初始带（例如从左、从右绕），各自优化，选总代价最小的。代价是计算量增加。

---

## 8. 手算：三位姿带的优化

### 8.1 设定（教学数值）

只有 3 个位姿：$s_0=(0,0)$，$s_1=(1,y)$，$s_2=(2,0)$；$y$ 是**唯一优化变量**。一个点障碍在 $(1,0)$。要求离障碍至少 $d_{min}+\epsilon=0.7$ m（即软约束）。速度上限 $v_{max}=1$ m/s，车在速度上限下尽量快，所以每段时间 $\Delta T=\ell/v_{max}=\ell$，其中 $\ell=\sqrt{1+y^2}$ 是段长。

简化的代价：

$$
J(y)=w_t\,T^2+w_o\,e_{obs}^2,\qquad T=2\ell=2\sqrt{1+y^2},\quad e_{obs}=\max(0,\ 0.7-|y|)
$$

取 $w_t=1$，$w_o=10$。（中点到障碍的距离就是 $|y|$。）

### 8.2 逐点计算

| $y$ | $\ell$ | $T=2\ell$ | $T^2$ | $e_{obs}$ | $10e_{obs}^2$ | $J$ |
|---|---|---|---|---|---|---|
| 0.0 | 1.000 | 2.000 | 4.000 | 0.7 | 4.900 | **8.900** |
| 0.3 | 1.044 | 2.088 | 4.360 | 0.4 | 1.600 | 5.960 |
| 0.4 | 1.077 | 2.154 | 4.640 | 0.3 | 0.900 | 5.540 |
| 0.5 | 1.118 | 2.236 | 5.000 | 0.2 | 0.400 | **5.400** |
| 0.6 | 1.166 | 2.332 | 5.440 | 0.1 | 0.100 | 5.540 |
| 0.7 | 1.221 | 2.441 | 5.960 | 0 | 0 | 5.960 |
| 0.8 | 1.281 | 2.561 | 6.560 | 0 | 0 | 6.560 |

### 8.3 解析验证

对 $0\le y\le0.7$：$T^2=4(1+y^2)$，所以

$$
J(y)=4(1+y^2)+10(0.7-y)^2,\qquad \frac{dJ}{dy}=8y-20(0.7-y)=28y-14=0\ \Rightarrow\ y^*=0.5
$$

$J(0.5)=4(1.25)+10(0.04)=5.0+0.4=5.4$ ✓，与表中一致。

### 8.4 读结果

1. 最优不是完全绕开（$y=0.7$，$J=5.96$），而是 $y=0.5$：**软约束**——为了少绕路（时间项），宁愿略微违反 0.2 m 的安全边距。这就是"罚函数"的特点。
2. 把障碍权重提到 $w_o=100$：$dJ/dy=8y-200(0.7-y)=208y-140=0\Rightarrow y^*=0.673$，更接近 0.7——**权重决定"软"到什么程度**；趋于无穷时逼近硬约束。
3. 若 $w_t$ 更大（更想快），$y^*$ 变小，更贴近障碍；这是"快"与"安全"的折中，也是调参的实质。
4. 实际 TEB 有几十个位姿、每个位姿三个变量，加上 $\Delta T$，规模更大但结构相同，靠稀疏求解器完成。

---

## 9. 从轨迹到控制指令；重规划

### 9.1 输出

优化后取**第一段**转成指令：

$$
v=\frac{\|p_1-p_0\|}{\Delta T_0},\qquad \omega=\frac{\beta_1-\beta_0}{\Delta T_0}
$$

**车式底盘**需要转向角：由 $\omega=v\tan\delta/L$ 反推 $\delta=\arctan(L\omega/v)$（teb 的参数 `cmd_angle_instead_rotvel` 就是输出转向角而不是角速度）。

**手算**（示例）：$v=1.0$ m/s，$\omega=0.179$ rad/s，$L=0.5$ m：$\delta=\arctan(0.5\times0.179/1.0)=\arctan(0.0895)=5.1°$。

CV 描述："outputting continuous pose and curvature references to the controller"，即把优化出的**位姿与曲率**序列作为参考交给下游 MPC 跟踪（见 [轨迹跟踪章](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)）。曲率可由 $\kappa_i=\omega_i/v_i$ 或相邻位姿的离散曲率得到。

### 9.2 重规划（receding horizon）

每个控制周期：

1. 更新局部代价地图、机器人当前位姿；
2. 以**上一周期的带**为初值（热启动 warm start），把起点更新为当前位姿，裁掉已走过的部分、补充新的局部目标；
3. 外迭代调整带长、内迭代优化；
4. 只执行第一段，下一周期重来。

**优点**：热启动让优化很快收敛；障碍变了，带自然"弹开"。

**风险**：

- 局部极小：凹形障碍前（死胡同）来回摆；同伦类并行可缓解；
- 目标附近振荡；
- 优化不收敛或不可行 → 输出无效 → 触发**恢复行为**（见 [导航架构章](./05_导航系统架构_move_base与Nav2.md)）。

---

## 10. 在本项目里怎么用

CV 的 P5 条目：*TEB local trajectory optimisation: optimised time-parameterised local trajectories under global-path and local-costmap constraints, outputting continuous pose and curvature references to the controller as local obstacles change.*

逐词：

| CV 词 | 本章对应 |
|---|---|
| time-parameterised local trajectories | 位姿 + $\Delta T$ 序列（第 5 节） |
| global-path constraints | 路径跟随 / via-point 项（第 6 节） |
| local-costmap constraints | 障碍项，局部代价地图（第 2、3 节） |
| continuous pose and curvature references | 优化后的位姿与曲率序列（第 9 节） |
| as local obstacles change | 每周期热启动重规划（第 9.2 节） |

在整条链里：Hybrid A* 给全局路径 → TEB 沿它做局部时间参数化优化 → MPC 跟踪 TEB 的输出。项目总览见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **【待你补全】** TEB 的关键参数（最大速度、倒车速度、加速度上限、最小转弯半径或轴距、障碍最小距离、`dt_ref`、权重）；是否用同伦类规划；TEB 输出给 MPC 的具体量（位姿序列、曲率、速度？）；调参时遇到的典型问题（如在狭窄通道来回摆）。CV 没有这些数值，请如实补全。

---

## 11. 常见错误与故障现象

| 现象 | 可能原因 |
|---|---|
| 车在狭窄通道里"摆头"、来回倒 | 膨胀过大；`min_obstacle_dist` 过大；局部极小；footprint 偏大 |
| 贴着障碍走 | 障碍权重小；膨胀小；footprint 偏小 |
| 规划频繁失败 / 无效轨迹 | 最小转弯半径设得比实际大；起点在膨胀区内；局部目标不可达 |
| 轨迹抖动 | 局部地图噪声；障碍闪烁；采样点数不稳定；`dt_ref` 太小 |
| 车跟不上规划速度 | 速度 / 加速度上限设得比底层能力高 |
| 不愿倒车 / 老是倒车 | 倒车速度上限与偏好前进权重设置不当 |
| 转向角突变 | 曲率约束权重小；加速度项小 |
| 撞上细杆、玻璃 | 传感器看不到，代价地图里没有 |
| 全局路径更新后车突然转向 | 局部目标跳变；全局路径重规划引起 |
| 全局代价地图和局部不一致 | 不同的膨胀参数 / 不同的障碍源 |

---

## 12. 面试怎么问、怎么答

**Q1. What is TEB and how does it work?**

> A: TEB, the Timed Elastic Band, represents the local trajectory as a sequence of poses with a time interval between each pair. I formulate a nonlinear least-squares problem: the cost includes following the global path, keeping a distance to obstacles, velocity and acceleration limits, the non-holonomic and minimum-turning-radius constraints, and a time-optimal term. The constraints are penalty functions, and the problem is solved with a sparse solver, g2o, every control cycle with a warm start from the previous band. I take the first segment as the command.

中文要点：位姿 + 时间间隔序列；非线性最小二乘；各代价项（路径、障碍、速度、加速度、运动学、最小半径、时间最优）都是罚函数；g2o 稀疏求解；每周期热启动，取第一段。

追问：为什么说它是软约束？（答：约束写成罚函数，权重有限时允许轻微违反；见三位姿手算。）为什么用超图？（答：速度边连 3 个顶点，加速度边连 5 个顶点，超边；稀疏性使求解高效。）

**Q2. Why TEB instead of DWA for a car-like robot?**

> A: DWA samples independent linear and angular velocities, which suits a differential drive. On a car-like platform the yaw rate is coupled to speed through the steering angle, and there is a minimum turning radius, so many DWA samples are not feasible. TEB has the non-holonomic and minimum-radius constraints inside the optimisation, it plans with time, and it can plan reverse motions.

**Q3. What do the costmap layers do, and what is inflation?**

> A: The static layer comes from the map, the obstacle layer from live sensor data with ray-tracing to clear cells, and the inflation layer expands obstacles with a cost that decays exponentially with distance. Cells within the inscribed radius are marked as certain collision. So the planner treats the robot as a point but still keeps a safe distance.

**Q4. What problems did you see with TEB, and how did you tune it?**（用自己的真实经历回答）

> A (template): The typical issues are oscillation in narrow passages and local minima. I looked at the obstacle distance and inflation first, then the weights of the obstacle and kinematics terms, and the velocity and acceleration limits, so the trajectory stayed within what the vehicle could track. 【请替换为你实际遇到的现象和调法】

**Q5. How does the local planner use the global path?**

> A: The global path acts as a reference: the trajectory is pulled towards it by the path-following term, and the local goal is a point on it ahead of the robot. The local planner is free to deviate when obstacles appear, and the global planner replans if the local one fails repeatedly.

## 13. 本章要点速记

1. 占据栅格用对数几率更新，加法形式；有上下限避免饱和。
2. 代价值：0 自由，253 内切障碍，254 致命，255 未知；膨胀 $252e^{-k(d-r_{ins})}$。
3. 全局代价地图整图更新慢，局部代价地图滑动窗口更新快。
4. 内切圆保证"任意朝向都碰"，外接圆保证"任意朝向都不碰"；长方形车两者差异大。
5. DWA：速度窗口 + 模拟 + 打分，不适合车式底盘（$\omega$ 与 $v$ 耦合）。
6. TEB 变量：位姿序列 $s_i$ + 时间间隔 $\Delta T_i$。
7. 代价项：路径、障碍、速度、加速度、非完整、最小半径、时间最优；全部罚函数 → 软约束。
8. 非完整项 $e_{nh}=(h_i+h_{i+1})\times d$：相邻位姿在同一圆弧上为零。
9. g2o 超图：顶点是位姿与 $\Delta T$，边是代价项；Hessian 稀疏。
10. 每周期热启动重规划，执行第一段；同伦类并行缓解局部极小。

## 自测题

1. 先验 $l_0=0$，三次命中（每次 $+0.85$）后 $p$？（答：$l=2.55$，$p=1/(1+e^{-2.55})=1/(1+0.0781)=0.927$。）
2. $r_{ins}=0.25$，$k=4$，$d=0.45$ 的膨胀代价？（答：$252e^{-4\times0.2}=252e^{-0.8}=252\times0.4493=113.2\approx113$。）
3. 速度窗口：$v_c=0.8$，$a_v=1.5$，$\Delta t=0.1$，求 $v$ 范围。（答：$[0.65,\ 0.95]$。）
4. 解释 TEB 里约束为什么用罚函数而不是硬约束？（要点：问题化为无约束最小二乘，可用稀疏 LM 快速求解；代价是约束可能被轻微违反，需权重和安全边距 $\epsilon$。）
5. $s_i=(0,0,0)$，$s_{i+1}=(1,1,\pi/2)$ 是否在同一圆弧上？计算 $e_{nh}$。（答：$(1+0)\cdot1-(0+1)\cdot1=0$，是——这是 $R=1$ 的四分之一圆。）
6. 三位姿带若 $w_t=2$（其他不变），求 $y^*$。（答：$J=8(1+y^2)+10(0.7-y)^2$，$dJ/dy=16y-20(0.7-y)=36y-14=0$，$y^*=0.389$。）
7. 为什么局部代价地图用滑动窗口？（要点：局部规划只需近处信息，窗口小、更新快、计算省，且随机器人移动。）

## 相关链接

- 上一章：[Hybrid A* 与车辆运动学约束](./02_Hybrid_A星与车辆运动学约束.md)
- 下一章：[轨迹跟踪：Pure Pursuit、Stanley、MPC](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)
- 相关：[导航系统架构：move_base 与 Nav2](./05_导航系统架构_move_base与Nav2.md)、[最优化与二次规划](../01_数学/04_最优化与二次规划.md)
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 八股：[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)
