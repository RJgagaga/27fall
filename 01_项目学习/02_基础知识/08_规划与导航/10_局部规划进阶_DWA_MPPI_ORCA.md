# 局部规划进阶：DWA、MPPI 与 ORCA

> **一句话定位**：局部规划器每个控制周期（通常 10–20 Hz）回答"下一小段时间我该用什么速度走"。前面已经学过 TEB（把轨迹当作优化变量）和 MPC 跟踪；本章补三种在工程和面试里同样常见的思路：**DWA**（在速度空间里采样、前向仿真、打分）、**MPPI**（在控制序列空间里大量采样、按代价指数加权平均，一种采样式的 MPC）、**ORCA**（多智能体/行人场景下，用速度障碍加线性约束做互惠避让）。重点是它们各自的原理、手算过程、优缺点，以及与 TEB、MPC 的取舍。

**本章学完你能：**

- 写出 DWA 的动态窗口、前向仿真、打分函数和"能刹住"约束，手算一组候选速度的评分与选择；
- 说出 DWA 的典型缺陷（局部极小、速度空间采样粗、对窄通道不友好）以及 Nav2 中 DWB 的"评论家（critic）"设计；
- 推导 MPPI 的更新公式 $u\leftarrow u+\sum_kw_k\delta u_k$，解释温度 $\lambda$ 与有效样本数，手算一次带权更新；
- 说明 MPPI 为什么能直接使用非凸代价（代价地图查表）和非线性模型，而 QP 型 MPC 不行，以及它为什么需要 GPU/并行；
- 解释速度障碍（VO）、互惠速度障碍（RVO）和 ORCA 的区别，手算一个双机迎面避让的 ORCA 半平面；
- 对比 TEB、DWA、MPPI、MPC 的适用场景，并把 P5 的 TEB 选择说清楚；
- 用英文回答 "How does DWA work / What are its limitations / How does MPPI work / What is ORCA and why reciprocal"。

**前置知识**：[代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)、[轨迹跟踪：Pure Pursuit、Stanley 与 MPC](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)、[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)、[车辆运动学](../02_力学与动力学建模/04_差速_麦克纳姆_全向轮与车辆运动学.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)。

## 目录

1. 局部规划要解决什么问题
2. 符号与单位
3. DWA：动态窗口法
4. DWA 的手算（含 Python 复核）
5. DWA 的缺陷与 Nav2 的 DWB
6. MPPI：采样式 MPC
7. MPPI 的推导
8. MPPI 的手算
9. MPPI 的实现要点
10. 速度障碍、RVO 与 ORCA
11. ORCA 的手算
12. 四种局部规划器的对比
13. 在本项目里怎么用
14. 常见错误与故障现象
15. 面试怎么问、怎么答
16. 本章要点速记
17. 自测题
18. 相关链接

---

## 1. 局部规划要解决什么问题

全局规划给出了从起点到目标的一条路径，但它是基于静态地图、低频更新的。机器人走着走着，会遇到：地图里没有的障碍、移动的行人、被占用的通道。**局部规划**在车体周围一个小范围内、较短的时间窗口里，结合最新的传感器数据，生成能跟随全局路径、避开障碍、满足运动学与动力学限制的短期运动。

```text
全局路径 + 局部代价地图 + 当前速度
            │
      局部规划器 (本章)
            │
   速度指令 (v, ω) 或 (v, δ)
            │
        底盘控制
```

局部规划器的三个基本要求：

1. **实时**：几十毫秒内给出结果；
2. **满足约束**：速度、加速度（动力学窗口）、非完整约束；
3. **反应式避障**：用最新传感器信息，不需要精确的全局知识。

不同局部规划器的差别可以归结为**"在哪个空间里搜索"**：

| 方法 | 搜索空间 | 核心做法 |
|---|---|---|
| DWA | 速度 $(v,\omega)$ 的二维窗口 | 枚举候选速度，前向仿真一小段轨迹，打分取最优 |
| TEB | 轨迹点与时间间隔 | 把整条轨迹作为优化变量，最小化加权代价 |
| MPC | 控制序列 | 在预测模型上求解带约束的优化 |
| MPPI | 控制序列 | 随机采样大量控制序列，指数加权平均 |
| ORCA | 速度平面 | 为每个邻居构造一个速度半平面约束，线性规划选最接近期望的速度 |

## 2. 符号与单位

| 符号 | 含义 | 单位 / 说明 |
|---|---|---|
| $v,\ \omega$ | 线速度、角速度 | m/s，rad/s |
| $v_a,\ \omega_a$ | 当前实际速度 | 同上 |
| $\dot v_{\max},\ \dot\omega_{\max}$ | 最大线加速度、角加速度 | m/s²，rad/s² |
| $\Delta t_w$ | 动态窗口的时间长度（一个控制周期或略长） | s |
| $T$ | 预测时域 | s |
| $G(v,\omega)$ | DWA 目标函数 | 无量纲 |
| $\mathbf u_{0:N-1}$ | MPPI 的控制序列 | 如 $(v,\omega)$ 的序列 |
| $\delta\mathbf u^{(k)}$ | 第 $k$ 个样本的控制扰动 | 同 $\mathbf u$ |
| $S^{(k)}$ | 第 $k$ 个样本的总代价 | 无量纲 |
| $\lambda$ | MPPI 温度参数 | 无量纲 |
| $w^{(k)}$ | 归一化权重 | $\sum w=1$ |
| $\mathbf p,\ \mathbf v$ | ORCA 中相对位置、相对速度 | m，m/s |
| $R$ | 两个智能体半径之和 | m |
| $\tau$ | ORCA 时间视界 | s |

## 3. DWA：动态窗口法

动态窗口法（Dynamic Window Approach，Fox 等，1997，细节未核对）的核心思想是：**不直接在整个速度空间里搜索，而是只考虑"下一个短时间内真的能达到的速度"**。

### 3.1 三类速度约束

1. **硬件限制**：$v\in[v_{\min},v_{\max}]$，$\omega\in[-\omega_{\max},\omega_{\max}]$；
2. **动态窗口**：受加速度限制，在时间 $\Delta t_w$ 内只能到达：

$$
v\in[v_a-\dot v_{\max}\Delta t_w,\ v_a+\dot v_{\max}\Delta t_w],\qquad\omega\in[\omega_a-\dot\omega_{\max}\Delta t_w,\ \omega_a+\dot\omega_{\max}\Delta t_w]
$$

3. **可刹停（admissible）**：选定速度后，如果前方有障碍，机器人必须还来得及减速到停下。若沿该轨迹到障碍物的距离为 $\text{dist}(v,\omega)$，要求

$$
v\le\sqrt{2\,\text{dist}(v,\omega)\,\dot v_{\max}}\ ,\qquad\omega\le\sqrt{2\,\text{dist}(v,\omega)\,\dot\omega_{\max}}
$$

这是 $v^2/(2\dot v_{\max})\le\text{dist}$（刹车距离不超过到障碍的距离）的变形。

三者的交集是**动态窗口**内的可行速度集合。

### 3.2 前向仿真

对窗口内的每一个候选 $(v,\omega)$，假设其在预测时域 $T$ 内保持不变，用运动模型积分得到一条短轨迹。以差速/单轮模型为例：

$$
\theta_{k+1}=\theta_k+\omega\,\Delta t,\qquad x_{k+1}=x_k+v\cos\theta_{k+1}\Delta t,\qquad y_{k+1}=y_k+v\sin\theta_{k+1}\Delta t
$$

对于车式（自行车模型）机器人，把 $\omega$ 换成 $v\tan\delta/L$ 即可，此时可采样的是 $(v,\delta)$，并要加上最小转弯半径限制。

### 3.3 打分

经典 DWA 的目标函数是三项加权和：

$$
G(v,\omega)=\sigma\bigl(\alpha\cdot\text{heading}(v,\omega)+\beta\cdot\text{dist}(v,\omega)+\gamma\cdot\text{vel}(v,\omega)\bigr)
$$

- $\text{heading}$：轨迹终点的朝向与目标方向的接近程度（越接近越好）；
- $\text{dist}$：轨迹与最近障碍的距离（越远越好）；
- $\text{vel}$：线速度（越快越好，鼓励前进）；
- $\sigma$：对三项做归一化平滑；$\alpha,\beta,\gamma$ 是权重。

选得分最大的 $(v,\omega)$ 下发，下一周期重新来一遍。这就是"反应式"：每个周期只执行第一步，所以决策是**短视的**。

## 4. DWA 的手算（含 Python 复核）

示例（非项目实测）。机器人当前速度 $v_a=0.5$ m/s，$\omega_a=0$，$\dot v_{\max}=1.0$ m/s²，$\dot\omega_{\max}=2.0$ rad/s²，$\Delta t_w=0.2$ s，$v_{\max}=1.0$。

**动态窗口**：

$$
v\in[0.5-0.2,\ 0.5+0.2]=[0.3,\ 0.7],\qquad\omega\in[0-0.4,\ 0+0.4]=[-0.4,\ 0.4]
$$

（Python 复核。）目标 $(4,1)$，障碍圆心 $(1.2,\,0)$，障碍半径与机器人半径之和 $R=0.3$（教学简化为单个膨胀圆），预测时域 $T=2$ s，$\Delta t=0.1$ s。权重 $\alpha=0.8,\ \beta=0.1,\ \gamma=0.1$。

heading 项定义：$(\pi-|\Delta|)/\pi$，其中 $\Delta$ 为轨迹终点处"机器人朝向"与"从终点指向目标的方向"之差，越接近 1 越好；dist 项取轨迹到障碍的最小间隙（上限 1 m）；速度项取 $v/0.7$。碰撞（间隙 $\le0$）的轨迹直接作废。

对窗口内部分候选，前向仿真得到（Python 复核）：

| $v$ | $\omega$ | 终点位置 | 碰撞？ | heading | 间隙 m | 得分 |
|---|---|---|---|---|---|---|
| 0.3 | 0.0 | (0.60, 0.00) | 否 | 0.909 | 0.300 | 0.800 |
| 0.3 | 0.2 | (0.58, 0.12) | 否 | 0.953 | 0.329 | 0.838 |
| 0.5 | 0.0 | (1.00, 0.00) | **是** | - | - | 作废 |
| 0.5 | 0.2 | (0.97, 0.21) | 否 | 0.954 | 0.008 | 0.836 |
| 0.5 | 0.4 | (0.89, 0.40) | 否 | 0.806 | 0.200 | 0.736 |
| 0.7 | 0.0 | (1.40, 0.00) | **是** | - | - | 作废 |
| 0.7 | 0.2 | (1.36, 0.29) | **是** | - | - | 作废 |
| 0.7 | 0.4 | (1.24, 0.56) | 否 | 0.796 | 0.092 | 0.746 |

观察：

1. 直行速度（$\omega=0$）在 2 秒内撞上障碍，被作废——说明预测时域内的碰撞检验让机器人**提前**转向；
2. 得分最高的是 $(0.3,\,0.2)$，得 0.838：慢速、向左小转；相邻的 $(0.5,\,0.2)$ 得分 0.836 几乎一样，但间隙只有 0.008 m，实际几乎擦着障碍——说明**要有足够的膨胀半径**，否则打分不会真正反映安全；
3. 速度项权重只有 0.1，所以 DWA 在障碍附近倾向于"减速绕行"，这符合预期。想让它更积极，可以提高 $\gamma$，但安全边际会下降。

实际实现里还要对三项在窗口内做归一化，并加上可刹停约束。本例在 2 秒时域中提前碰撞检验已经起到了避障作用。

## 5. DWA 的缺陷与 Nav2 的 DWB

| 缺陷 | 原因 | 缓解 |
|---|---|---|
| 局部极小 | 只有短视的启发（朝目标 + 障碍），遇到凹形障碍会卡住 | 依赖全局路径引导；用路径距离代替直线距离作为 heading 项 |
| 恒速假设 | 一条轨迹内速度不变，不如 TEB/MPC 的变速轨迹灵活 | 缩短预测时域；增大采样密度 |
| 速度空间采样粗 | 窗口很小，采样步长大会漏掉狭窄可行区 | 增加采样数；多分辨率 |
| 窄通道通过性差 | 圆弧轨迹族不容易沿狭窄走廊的直线通过 | 配合全局路径跟随项；使用 TEB |
| 权重敏感 | 三项权重需要手动调，不同场景表现差异大 | 引入更多"评论家"逐项调参 |
| 不适合非完整的车式机器人 | 默认差速模型，采样 $(v,\omega)$ 与阿克曼的最小转弯半径冲突 | 采样 $(v,\delta)$ 并裁剪 |

Nav2 提供的 **DWB 控制器**（`nav2_dwb_controller`，来自 ROS1 中 `dwa_local_planner` 的重构，细节未核对）把 DWA 的打分拆成一组可配置的"**评论家（critic）**"插件，每个评论家对一条轨迹给一个分，总分是加权和，例如：

- 对路径对齐（`PathAlign`）、对路径距离（`PathDist`）、对目标距离（`GoalDist`）、对目标朝向（`GoalAlign`）；
- 障碍物足迹代价（`ObstacleFootprint`）；
- 转向（`RotateToGoal`）、扭曲（`Twirling`）、慢速接近目标（`PreferForward`）等。

这让 DWA 的权重与行为更模块化，并允许用全局路径作为主要引导，缓解局部极小。这些评论家名称按记忆列出，以所用发行版文档为准。

## 6. MPPI：采样式 MPC

模型预测路径积分（Model Predictive Path Integral, MPPI，Williams 等，2016–2017，细节未核对）是一种基于采样的 MPC：**不求解优化问题的梯度，而是随机采样大量的控制序列，用前向仿真评价代价，再用代价的指数加权平均更新控制**。

与 QP 型 MPC（见 [MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)）相比：

| 维度 | QP/NLP 型 MPC | MPPI |
|---|---|---|
| 求解方式 | 梯度/QP 求解器，局部凸近似 | 随机采样 + 加权平均，无需梯度 |
| 模型 | 需要线性化或可微非线性 | 任意黑盒前向仿真 |
| 代价 | 要求光滑、可微（最好凸） | 任意（含代价地图查表、碰撞指示函数） |
| 约束 | 可显式写不等式约束 | 通过惩罚项或截断实现，不严格保证 |
| 算力 | 少量大矩阵运算 | 大量独立并行仿真，适合 GPU/多核 |
| 易错点 | 非凸、不可行 | 噪声、样本数、温度参数 |

## 7. MPPI 的推导

### 7.1 问题

系统 $\mathbf x_{t+1}=f(\mathbf x_t,\mathbf u_t)$。要找控制序列 $U=(\mathbf u_0,\dots,\mathbf u_{N-1})$ 最小化期望代价 $S(U)=\phi(\mathbf x_N)+\sum_t q(\mathbf x_t)+\tfrac12\mathbf u_t^\top R\,\mathbf u_t$。

### 7.2 采样

设当前"名义"控制序列为 $U$，在其上加高斯噪声：$V^{(k)}=U+\delta U^{(k)}$，$\delta\mathbf u_t^{(k)}\sim\mathcal N(0,\Sigma)$，$k=1,\dots,K$。对每个样本前向仿真得到轨迹和总代价 $S^{(k)}$。

### 7.3 最优分布与权重

路径积分控制理论（来自随机最优控制，细节从略）指出：最优控制序列的分布与"噪声分布 × $\exp(-S/\lambda)$"成正比。用重要性采样，把这个目标分布下的期望控制近似为样本的加权平均：

$$
w^{(k)}=\frac{\exp\bigl(-\frac1\lambda(S^{(k)}-\beta)\bigr)}{\sum_j\exp\bigl(-\frac1\lambda(S^{(j)}-\beta)\bigr)},\qquad\beta=\min_kS^{(k)}
$$

减去最小代价 $\beta$ 不改变归一化结果，但避免了指数下溢（数值稳定技巧）。

### 7.4 更新

$$
\mathbf u_t\leftarrow\mathbf u_t+\sum_{k=1}^Kw^{(k)}\,\delta\mathbf u_t^{(k)}
$$

也就是说，新控制 = 旧控制 + 按代价加权的扰动平均。执行第一步 $\mathbf u_0$，然后把序列整体左移一步（把末尾补上），下个周期继续（滚动时域）。

### 7.5 温度参数的含义

- $\lambda\to0$：权重几乎全集中在代价最小的那个样本，退化成"选最优样本"，对噪声敏感；
- $\lambda\to\infty$：权重趋于均匀，更新变成所有扰动的简单平均，趋近于 0，几乎不更新；
- 合适的 $\lambda$ 使得**有效样本数** $N_{\text{eff}}=1/\sum_kw_k^2$ 处于一个适中值（既不是 1，也不是 $K$）。

## 8. MPPI 的手算

示例（非项目实测）：一个 1D 点质量，状态 $x$，控制 $u$ 直接是位移（$x_{t+1}=x_t+u_t$，$\Delta t=1$），$N=3$ 步，初始 $x_0=0$，目标是在 $t=3$ 时到达 $x=1$。代价 $S=(x_3-1)^2+0.1\sum_tu_t^2$。名义控制 $U=(0,0,0)$，$\lambda=1$，噪声标准差 0.5，取 $K=6$ 个样本（随机种子固定，Python 复核）。

样本扰动 $\delta U^{(k)}$ 与代价（保留三位小数）：

| $k$ | $\delta u_0$ | $\delta u_1$ | $\delta u_2$ | 终点 $x_3$ | 代价 $S$ | 权重 $w$ |
|---|---|---|---|---|---|---|
| 1 | 1.020 | −1.278 | 0.209 | −0.049 | 1.371 | 0.2143 |
| 2 | −0.284 | −0.226 | −0.108 | −0.618 | 2.632 | 0.0607 |
| 3 | −1.010 | −0.116 | −0.433 | −1.559 | 6.668 | 0.0011 |
| 4 | 1.661 | 0.113 | −0.176 | 1.598 | 0.638 | 0.4458 |
| 5 | −0.141 | −0.334 | −0.528 | −1.003 | 4.050 | 0.0147 |
| 6 | −0.195 | 0.241 | −0.119 | −0.073 | 1.164 | 0.2635 |

最小代价 $\beta=0.638$（样本 4）。权重例：样本 1 的 $\exp(-(1.371-0.638)/1)=0.480$，权重分母 $\sum\exp(\cdot)=2.24$，故 $w_1=0.480/2.24=0.214$。有效样本数：$N_{\text{eff}}=1/\sum w^2=3.15$（Python 复核）——6 个样本里约 3 个有实际贡献。

加权更新：

$$
\mathbf u_{\text{new}}=\sum_kw^{(k)}\delta U^{(k)}=(0.887,\ -0.179,\ -0.080)
$$

对应终点 $x_3=0.887-0.179-0.080=0.629$（Python 复核）。一次迭代后，期望终点从 0 移到 0.629，朝目标 1 前进了一大步（真实最优是 $\sum u\approx0.9$ 附近，因为控制代价抵消一部分）。再多迭代几次，或增大样本数，会更接近。

注意第一个分量 0.887 远大于其他：因为只有 $u_0$ 大的样本（样本 1、4）代价小，$u_1,u_2$ 方向的信息被噪声淹没，这正是实际中要采用较大样本数（成百上千）和平滑滤波的原因。

## 9. MPPI 的实现要点

1. **并行采样**：样本之间完全独立，天然适合多线程/GPU；Nav2 提供的 MPPI 控制器（`nav2_mppi_controller`，Humble 起提供，细节未核对）以 CPU SIMD 并行为主，样本数通常在数百到两千；
2. **代价函数由多个"评论家"组成**：路径跟随、目标距离、障碍物（直接查代价地图）、速度/转向约束惩罚、"优先前进"等，思想与 DWB 类似；
3. **控制平滑**：直接加权平均仍有噪声，常对输出序列做 Savitzky–Golay 滤波等平滑，减少抖动；
4. **约束处理**：控制量限幅在采样后截断；状态约束（如不进入障碍）写成大惩罚项；严格保证安全需要外层安全监控；
5. **模型**：可以是差速、全向或阿克曼（带最小转弯半径）；车式机器人要采样 $(v,\delta)$ 或 $(v,\dot\delta)$；
6. **温度 $\lambda$ 与噪声 $\Sigma$** 需要联合调参：噪声小则探索不足，噪声大则大量样本落在高代价区，$N_{\text{eff}}$ 过低；
7. **热启动**：上周期的序列左移作为下周期的名义序列，大大提高效率。

> **提示**：MPPI 的"魔法"在于**代价函数可以任意非凸**——直接查询栅格地图、写 if-else 的碰撞判定都行。它的代价是随机性和样本数需求，以及不严格满足约束。

## 10. 速度障碍、RVO 与 ORCA

DWA、MPPI 把别的移动障碍物当成"预测轨迹上的障碍"。当场景里有多个**会主动避让的智能体**（行人、其它机器人）时，更好的方法是在**速度空间**里推理：

### 10.1 速度障碍（Velocity Obstacle, VO）

对机器人 A（半径 $r_A$）和移动障碍 B（半径 $r_B$），令 $R=r_A+r_B$。定义 A 相对 B 的速度 $\mathbf v=\mathbf v_A-\mathbf v_B$，相对位置 $\mathbf p=\mathbf p_B-\mathbf p_A$。如果相对速度保持不变，在未来 $\tau$ 内发生碰撞，当且仅当相对速度落在一个**锥形区域**内（从 A 指向 B、被两条切线夹住的锥，并在 $\tau$ 处被截断）。这个区域叫作**速度障碍**。A 只要选择一个速度，使得相对速度**不落在** VO 内，就能保证 $\tau$ 内不碰撞（在 B 保持速度的假设下）。

### 10.2 振荡问题与 RVO

如果两个机器人都用 VO，各自都以为对方保持原速，各自选一个避让速度，结果常常两个都往同一侧躲，然后又一起躲回来，**来回振荡**。互惠速度障碍（Reciprocal Velocity Obstacle, RVO，van den Berg 等，2008，细节未核对）让双方各承担一半的避让责任，减小振荡。

### 10.3 ORCA

最优互惠碰撞避免（Optimal Reciprocal Collision Avoidance, ORCA，van den Berg 等，2011，细节未核对）把 RVO 进一步变成**线性约束**：

1. 对每个邻居，找到相对速度 $\mathbf v$ 到 VO 边界的**最近点**，记从 $\mathbf v$ 到该点的向量为 $\mathbf u$（"最小需要改变量"），边界在该点的外法向量为 $\mathbf n$；
2. 两个智能体**各承担一半**：A 的允许速度集合是一个半平面

$$
\text{ORCA}_{A|B}=\Bigl\{\mathbf v':\ \bigl(\mathbf v'-(\mathbf v_A+\tfrac12\mathbf u)\bigr)\cdot\mathbf n\ge0\Bigr\}
$$

3. A 对每个邻居都得到一个半平面，再加上最大速度圆盘约束，求解**线性规划**：在所有半平面交集内选离"期望速度"最近的速度。

因为是半平面，所以计算复杂度对邻居数线性，实时性很好，常用于人群仿真和多机器人避让（如 RVO2 库）。

## 11. ORCA 的手算

示例（非项目实测，Python 复核）。机器人 A 在 $(0,0)$，速度 $(1,0)$ m/s；机器人 B 在 $(4,\,0.3)$，速度 $(-1,0)$ m/s，近似迎面但略偏。两者半径均为 0.5，$R=1.0$ m，时间视界 $\tau=3$ s。

- 相对位置 $\mathbf p=\mathbf p_B-\mathbf p_A=(4,\,0.3)$，距离 4.011 m；
- 相对速度 $\mathbf v=\mathbf v_A-\mathbf v_B=(2,\,0)$；
- 截断锥：圆心 $\mathbf p/\tau=(1.333,\,0.1)$，半径 $R/\tau=0.333$。

相对速度 $(2,0)$ 在锥内（因为 $|\mathbf v|$ 超过截断圆，且方向差在切线夹角 $\arcsin(R/|\mathbf p|)=14.4^\circ$ 之内，而 $\mathbf p$ 方向为 4.3°，$\mathbf v$ 方向为 0°），所以会碰撞，需要避让。

计算最近边界点。$\mathbf v$ 位于圆心"外侧"（沿 $\mathbf p$ 方向超过了圆心：$\mathbf w=\mathbf v-\mathbf p/\tau=(0.667,\,-0.1)$，$\mathbf w\cdot\mathbf p=2.64>0$），因此最近点落在锥的某条腿（切线）上。切线方向与 $\mathbf p$ 夹角 $\pm14.4^\circ$：

- 右腿方向 $\mathbf d_R=(0.984,\,-0.176)$：$\mathbf v\cdot\mathbf d_R=1.969$，投影 $=1.969\,\mathbf d_R=(1.938,\,-0.347)$，$\mathbf u_R=(1.938-2,\ -0.347-0)=(-0.062,\ -0.347)$，$\lVert\mathbf u_R\rVert=0.352$；
- 左腿方向 $\mathbf d_L=(0.947,\,0.321)$：投影后 $\mathbf u_L=(-0.206,\,0.608)$，$\lVert\mathbf u_L\rVert=0.642$。

右腿更近，取 $\mathbf u=(-0.062,\,-0.347)$。法向量 $\mathbf n=\mathbf u/\lVert\mathbf u\rVert=(-0.176,\,-0.984)$。

**A 的半平面**：$\bigl(\mathbf v'-(\mathbf v_A+\tfrac12\mathbf u)\bigr)\cdot\mathbf n\ge0$，其中 $\mathbf v_A+\tfrac12\mathbf u=(0.969,\,-0.174)$。A 的期望速度 $(1,0)$ 代入：$(0.031,\,0.174)\cdot\mathbf n=-0.177<0$，违反约束，需要投影到半平面边界：

$$
\mathbf v_A'=(1,0)+0.177\,\mathbf n=(0.969,\ -0.174)
$$

**B 的半平面**对称（反向分担）：$\mathbf v_B'=\mathbf v_B-\tfrac12\mathbf u=(-0.969,\ 0.174)$。

结果：A 向自己的右前方（$-y$）微调，B 从自己朝向 $-x$ 的视角也向其右侧（$+y$）微调；二者**同时、各让一半**，相对速度变为 $(1.938,\,-0.347)$，恰在 VO 边界上，未来 3 秒内不再碰撞。若只有 A 单方面避让（VO 而非 ORCA），则 A 要承担全部的 $\mathbf u$（速度变化 0.352 m/s）；ORCA 下每人只需改变 0.176 m/s，机动幅度减半，且两者的选择**自洽**（都向自己的右侧），不会振荡。

## 12. 四种局部规划器的对比

| 维度 | DWA/DWB | TEB | MPC | MPPI | ORCA |
|---|---|---|---|---|---|
| 搜索空间 | 速度窗口 | 轨迹点 + 时间间隔 | 控制序列（连续优化） | 控制序列（随机采样） | 速度平面 |
| 是否最优化 | 打分选优（有限候选） | 局部最优化 | 局部最优化 | 近似（采样） | 线性规划 |
| 运动学适配 | 差速为主，车式需改造 | 支持阿克曼最小转弯半径 | 可显式建模 | 任意模型 | 全向/圆盘模型 |
| 非凸代价 | 容易 | 一般（图优化） | 难 | 容易 | 不适用 |
| 动态障碍 | 预测轨迹碰撞检测 | 支持（时间域） | 需预测 | 需预测 | 专长（互惠避让） |
| 计算开销 | 小 | 中 | 中到大 | 大（并行） | 极小 |
| 典型问题 | 局部极小、窄通道 | 参数多、收敛依赖初值 | 非凸可行性 | 噪声、样本数 | 假设圆形、速度可瞬变 |
| 典型场景 | 简单室内差速车 | 车式/窄环境 | 精确跟踪 | 复杂代价、高机动 | 人群/多机器人 |

实际系统往往**分层组合**：全局规划 → 局部规划（TEB/DWB/MPPI）→ 底层跟踪（PID/MPC）；在人群中再叠加 ORCA 之类的速度层避让。

## 13. 在本项目里怎么用

- **P5 车式导航**：P5 的局部规划使用 **TEB**，MPC 做轨迹跟踪。选择 TEB 而不是 DWA 的合理解释：(1) 车式底盘有最小转弯半径，TEB 的优化里可以直接加转弯半径约束，DWA 的 $(v,\omega)$ 采样对非完整车式适配不自然；(2) TEB 优化的是带时间的整条轨迹，输出连续的位姿和曲率参考，正好给 MPC 当参考（见 CV：TEB "输出连续位姿和曲率参考给控制器"）；(3) 窄通道里 TEB 的表现更好。面试中如果被问到"为什么不用 DWA 或 MPPI"，要围绕这几点回答。DWA、MPPI 的内容属于**延伸知识**，不是 P5 实现的部分。见 [TEB 局部轨迹优化](../../01_项目课程/P5_车式移动机器人自主导航/09_TEB局部轨迹优化.md)、[MPC 轨迹跟踪](../../01_项目课程/P5_车式移动机器人自主导航/10_MPC轨迹跟踪.md)、[代价地图与车体外形](../../01_项目课程/P5_车式移动机器人自主导航/07_代价地图与车体外形.md)。
- **MPC 与 MPPI 的关系**：P5 的 MPC 在车式运动学模型上做滚动优化（约束位置、航向、速度、转角）。MPPI 是同一滚动时域思想的采样版，MPC 的约束处理更严格，MPPI 的代价形式更自由——被问到"两者区别"时按第 6 节表格回答。
- **P6 多机器人**：ORCA 一类互惠避让适合多机器人场景；P6 做的是平台协同（发现、状态同步、任务分发），没有实现 ORCA。见 [DDS 与 M-DDS 分布式协同](../../01_项目课程/P6_深开鸿M-Robots平台软件/06_DDS与M-DDS分布式协同.md)。
- **P3 / P1 / P7**：不涉及局部路径规划。

诚实表述：我的车式导航项目用 TEB 做局部轨迹优化、MPC 做跟踪；DWA 和 MPPI 我了解其原理和适用场景，但没有在项目里实现或部署，不会声称用过。

## 14. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| DWA 在凹形障碍前反复摇摆 | 局部极小 | 依赖全局路径引导；增加路径跟随项；触发恢复行为 |
| 机器人擦着障碍物走 | 膨胀半径不足；打分对间隙不够敏感 | 增大膨胀；提高障碍项权重或设最小间隙硬约束 |
| 机器人在障碍前明显减速、过于保守 | 速度项权重低；刹停约束保守 | 调整权重；检查加速度参数是否过小 |
| 窗口很小，几乎不能转向 | 加速度限制或时间窗过短 | 核对 $\dot v_{\max},\dot\omega_{\max}$ 与控制周期 |
| 车式机器人轨迹违反最小转弯半径 | 采样 $(v,\omega)$ 未考虑转角限制 | 采样 $(v,\delta)$ 或裁剪 $\omega\le v\tan\delta_{\max}/L$ |
| MPPI 输出抖动 | 样本不足；噪声大；无平滑 | 增大样本数；降低噪声；输出滤波 |
| MPPI 几乎不更新 | $\lambda$ 过大，权重近乎均匀 | 减小 $\lambda$，检查 $N_{\text{eff}}$ |
| MPPI 总是选单个样本，行为不稳 | $\lambda$ 过小，$N_{\text{eff}}\approx1$ | 增大 $\lambda$ 或样本数 |
| MPPI 的控制周期超时 | 样本数 × 预测步数过大 | 降低样本数/时域；使用并行或降低代价地图查询频率 |
| ORCA 下机器人偶尔相撞 | 机器人不是圆盘、速度不能瞬变；邻居信息延迟 | 加大半径余量；考虑加速度限制与延迟补偿 |
| 多机器人在狭窄处互相堵死 | ORCA 是局部方法，没有全局协调 | 在更高层使用 MAPF 或交通规则（见上一章） |

## 15. 面试怎么问、怎么答

**Q1. Explain how the Dynamic Window Approach works.**

> At every control cycle, I compute the window of velocities the robot can reach within a short time given its acceleration limits. I sample candidate pairs of linear and angular velocity in that window, simulate each one forward for a second or two with a constant-velocity assumption, discard trajectories that collide or that I couldn't stop in time, and score the rest with a weighted sum of heading toward the goal, clearance and speed. I send the best one and repeat next cycle.

中文：每周期先算加速度限制下可达的速度窗口，采样 $(v,\omega)$，前向仿真 1–2 秒，丢弃碰撞或刹不住的，对剩余按朝向、间隙、速度加权打分，执行最优并下周期重来。追问：刹停约束是什么？（$v\le\sqrt{2\,\text{dist}\,\dot v_{\max}}$。）

**Q2. What are the weaknesses of DWA, and why might you choose TEB or MPPI instead?**

> DWA assumes constant velocity along each short trajectory, only searches a small velocity window and is greedy, so it can get stuck in local minima such as concave obstacles and does poorly in narrow corridors. It also fits differential-drive robots better than car-like ones. TEB optimises a whole time-parameterised trajectory with kinematic constraints, which suits car-like robots and narrow passages, and MPPI can use arbitrary non-convex costs and models through sampling.

中文：DWA 恒速假设、窗口小、短视，凹障碍易卡、窄通道差、更适合差速；TEB 优化整条带时间轨迹且能加运动学约束，适合车式和窄通道；MPPI 靠采样可用任意非凸代价与模型。

**Q3. How does MPPI compute its control?**

> I take the current nominal control sequence, add Gaussian noise to create many perturbed sequences, simulate each with the dynamics model, and compute a cost. Then I weight each perturbation by the exponential of its negative cost divided by a temperature, normalise, and add the weighted average perturbation to the nominal sequence. I apply the first control and shift the sequence for the next cycle. It needs no gradients, so the cost can be non-convex, such as a lookup in the costmap.

中文：在名义控制序列上加高斯噪声采样很多序列，前向仿真算代价，按 $\exp(-S/\lambda)$ 加权归一化，把加权平均扰动加到名义序列上，执行第一步并左移；不需要梯度，所以代价可非凸，比如查代价地图。追问：$\lambda$ 太大或太小会怎样？（太大几乎不更新，太小只选单个样本。）

**Q4. Why is ORCA reciprocal, and what does that buy you?**

> In plain velocity obstacles, each agent assumes the other keeps its velocity, so two agents can both dodge to the same side and oscillate. In ORCA each agent takes half of the smallest velocity change needed to leave the collision cone, which gives each of them a half-plane of allowed velocities. They both follow the same rule, so their choices are consistent, the oscillation disappears, and the problem becomes a small linear program.

中文：普通 VO 假设对方不变，双方可能躲到同侧并振荡；ORCA 每方承担最小速度改变量的一半，得到允许速度半平面，规则一致、无振荡，问题变成小的线性规划。

**Q5. Why did you use TEB and MPC rather than DWA in your navigation project?**

> My robot is car-like, so it has a minimum turning radius and the heading matters. TEB optimises a time-parameterised trajectory under those kinematic constraints and gives continuous pose and curvature references, which the MPC then tracks. DWA samples velocities in a small window and fits differential-drive robots better. TEB also behaves better in narrow passages, where I needed the planner to find a smooth, feasible path.

中文：我的车式机器人有最小转弯半径且航向重要；TEB 在运动学约束下优化带时间轨迹并给 MPC 连续位姿和曲率参考；DWA 小窗口采样更适合差速；窄通道里 TEB 更平滑可行。

## 16. 本章要点速记

1. 局部规划器的差别在搜索空间：DWA 搜速度、TEB 搜轨迹、MPC/MPPI 搜控制序列、ORCA 搜速度平面。
2. DWA 窗口：$v\in[v_a\pm\dot v_{\max}\Delta t_w]$，$\omega$ 同理；刹停约束 $v\le\sqrt{2\,\text{dist}\,\dot v_{\max}}$。
3. DWA 打分 = 朝向 + 间隙 + 速度的加权和；轨迹碰撞则作废。
4. DWA 短视、恒速，易局部极小，窄通道和车式适配差；Nav2 DWB 用可配置评论家缓解。
5. MPPI：采样 $\delta U$，$w\propto\exp(-(S-\beta)/\lambda)$，$U\leftarrow U+\sum w\,\delta U$。
6. $\lambda$ 小则只选最优样本，$\lambda$ 大则几乎不更新；用 $N_{\text{eff}}=1/\sum w^2$ 检查。
7. MPPI 的优势：任意非凸代价、黑盒模型、天然并行；缺点：随机、不严格保证约束。
8. VO：相对速度落在碰撞锥内会碰；RVO/ORCA 让双方各承担一半，避免振荡。
9. ORCA：每个邻居一个半平面 $(\mathbf v'-(\mathbf v_A+\tfrac12\mathbf u))\cdot\mathbf n\ge0$，线性规划求最近速度。
10. P5 用 TEB + MPC 不是 DWA/MPPI；DWA、MPPI、ORCA 属延伸知识。

## 17. 自测题

1. 当前 $v_a=0.8$，$\dot v_{\max}=0.5$ m/s²，$\Delta t_w=0.4$ s，$v_{\max}=1.0$，线速度窗口是多少？（$[0.6,\,1.0]$，上限恰好等于 $v_{max}=1.0$。）
2. $v=1$ m/s，$\dot v_{\max}=1$ m/s²，前方障碍距离 0.4 m，满足刹停约束吗？（$\sqrt{2\times0.4\times1}=0.894<1$，不满足。）
3. MPPI 三个样本代价 1、2、3，$\lambda=1$，权重大约是多少？（$e^{0},e^{-1},e^{-2}=1,0.368,0.135$，和 1.503，权重 $0.665,0.245,0.090$。）
4. 为什么 MPPI 要减去 $\beta=\min S$？（数值稳定，避免 $\exp(-S/\lambda)$ 下溢；不影响归一化后的权重。）
5. ORCA 相对 VO 的关键改进是什么？（互惠：双方各承担一半避让量，构成半平面约束，消除振荡并可用线性规划求解。）

## 18. 相关链接

- 上一章：[多机器人路径规划：MAPF 与编队](./09_多机器人路径规划_MAPF_编队.md)
- 前置：[代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)、[轨迹跟踪：Pure Pursuit、Stanley 与 MPC](./04_轨迹跟踪_PurePursuit_Stanley_MPC.md)
- 控制：[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)
- 项目：[P5 车式移动机器人导航 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 模块说明：[08_规划与导航 README](./README.md)
