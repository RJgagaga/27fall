# 最优控制与轨迹优化：iLQR、DDP 与直接配点

> **一句话定位**：LQR 回答"线性系统、二次代价、无约束"下的最优反馈；现实问题往往是非线性的、带约束的、只关心一段有限时间内的轨迹——倒立摆起摆、机械臂最短时间搬运、轮腿跳跃与恢复。轨迹优化（trajectory optimisation）把"找一条最优的状态—控制轨迹"写成优化问题，iLQR/DDP 沿用 LQR 的递推思路，直接配点把问题变成一个大规模非线性规划；MPC 就是把它们放进滚动时域里在线求解。

**本章学完你能：**

- 写出连续与离散最优控制问题的标准形式，用 Pontryagin 最小值原理推出必要条件，并手算双积分器最小能量问题（$u(t)=6-12t$，代价 $12$）；
- 用动态规划推出 LQR 的 Riccati 递推，理解价值函数（value function）的二次近似；
- 不跳步地推导 iLQR 的反向递推（$Q$ 函数展开、$k$ 与 $K$）与前向滚动，手算一个标量非线性系统的完整一次迭代并看它收敛；
- 说清 DDP 与 iLQR 的区别（是否保留动力学二阶项），以及正则化与线搜索的作用；
- 区分单打靶、多重打靶与直接配点（trapezoidal / Hermite–Simpson）；用数字说明单打靶对不稳定系统的灵敏度爆炸，并看离散化步数对配点解的逼近；
- 理解约束如何处理，以及 MPC 实时迭代（RTI）、热启动、凝聚（condensing）等工程技巧；
- 用英文回答 "iLQR vs LQR vs MPC / shooting vs collocation / why warm start" 等问题。

**前置知识**：[LQR 最优控制](./04_LQR最优控制.md)（Riccati、Q/R）、[MPC 模型预测控制](./05_MPC模型预测控制.md)（预测时域、QP）、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)（KKT、牛顿法）、[微积分与常微分方程](../01_数学基础/02_微积分与常微分方程.md)（线性化）、[数值线性代数与稀疏求解](../01_数学基础/08_数值线性代数与稀疏求解.md)（稀疏 KKT 系统）。

## 目录

1. 问题：从 LQR 走向非线性、有约束的轨迹优化
2. 符号与单位
3. 最优控制问题的标准形式
4. 变分法与 Pontryagin 最小值原理
5. 动态规划与 Riccati 的来历
6. iLQR：推导
7. 手算：标量系统的 iLQR 全过程
8. DDP、正则化与线搜索
9. 打靶法与直接配点
10. 约束的处理
11. MPC 的求解器与实时迭代
12. 在本项目里怎么用
13. 常见错误与故障现象
14. 面试怎么问、怎么答
15. 本章要点速记
16. 自测题
17. 相关链接

---

## 1. 问题：从 LQR 走向非线性、有约束的轨迹优化

先看三个任务。

**倒立摆起摆**：摆杆从下垂位置甩到竖直向上，这个过程里角度变化很大，$\sin\theta\ne\theta$，线性化模型完全失效，而且力矩有上限。LQR 只能在竖直点附近镇住，不能"摆上去"。

**机械臂时间最优搬运**：从 A 到 B，要最快，同时每个关节的速度、力矩都有上限，还要避开障碍。目标是"时间"，约束是各种不等式。

**轮腿机器人落地缓冲与恢复**：腿长在大范围变化，质心高度变化，接触条件切换（离地—触地）。这是一个明显的非线性、带约束、带接触切换的问题。

这些问题有共同点：**状态与控制的整条轨迹是未知数，要在满足动力学（等式约束）和物理限制（不等式约束）的前提下，使某个代价最小**。这就是轨迹优化。LQR 是它在"线性动力学 + 二次代价 + 无约束"下的闭式解；其余情形需要数值方法。

本章分三层：（1）理论基础：Pontryagin 与动态规划；（2）基于动态规划的数值方法：iLQR/DDP；（3）基于非线性规划的方法：打靶与配点。最后讨论 MPC 如何实时地求解它们。

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $x_k\in\mathbb{R}^{n}$ | 第 $k$ 步的状态 | 视状态 |
| $u_k\in\mathbb{R}^{m}$ | 第 $k$ 步的控制 | 视控制 |
| $f(x,u)$ | 离散动力学 $x_{k+1}=f(x_k,u_k)$ | |
| $\ell(x,u)$ | 阶段代价 | |
| $\ell_N(x)$ | 终端代价 | |
| $J$ | 总代价 | |
| $V_k(x)$ | 价值函数（cost-to-go） | |
| $Q_k(x,u)$ | 动作价值函数 | |
| $A_k=f_x,\ B_k=f_u$ | 动力学对状态/控制的雅可比 | |
| $V_x,V_{xx}$ | 价值函数的一、二阶导数 | |
| $k_k,K_k$ | 前馈增量、反馈增益（iLQR 的输出） | |
| $\lambda$ | 协态（costate，拉格朗日乘子） | |
| $H$ | 哈密顿量 | |
| $N$ | 预测/优化时域步数 | |

## 3. 最优控制问题的标准形式

**连续时间**：

$$\min_{u(\cdot)}\ J=\ell_f(x(T))+\int_0^T\ell(x(t),u(t),t)\,dt\quad\text{s.t.}\quad\dot x=f(x,u),\ x(0)=x_0,\ g(x,u)\le0.$$

**离散时间**（数值计算用）：

$$\min_{x_{0:N},u_{0:N-1}}\ J=\ell_N(x_N)+\sum_{k=0}^{N-1}\ell(x_k,u_k)\quad\text{s.t.}\quad x_{k+1}=f(x_k,u_k),\ x_0\text{ 给定},\ g(x_k,u_k)\le0.$$

两种基本的“变量选择”：（1）只把控制 $u$ 当变量，状态由动力学滚动（**打靶，shooting**）；（2）把 $x$ 和 $u$ 都当变量，动力学作为等式约束（**配点，collocation / 直接转录**）。iLQR 属于前者的一种（利用动态规划结构）。

## 4. 变分法与 Pontryagin 最小值原理

### 4.1 哈密顿量

引入协态 $\lambda(t)$，定义哈密顿量

$$H(x,u,\lambda)=\ell(x,u)+\lambda^\top f(x,u).$$

**Pontryagin 最小值原理（无状态约束时）**：最优轨迹 $(x^\ast,u^\ast)$ 满足

$$\dot x=\frac{\partial H}{\partial\lambda}=f,\qquad \dot\lambda=-\frac{\partial H}{\partial x},\qquad u^\ast=\arg\min_uH(x^\ast,u,\lambda),\qquad \lambda(T)=\frac{\partial\ell_f}{\partial x}.$$

当 $u$ 无约束且 $H$ 光滑时，最后一个条件化为 $\partial H/\partial u=0$。直观：$\lambda$ 是"状态的影子价格"，反向（从终点往回）积分，告诉你状态偏一点点，终端代价会差多少。

### 4.2 手算：双积分器最小能量

系统 $\dot x_1=x_2,\ \dot x_2=u$，从 $x(0)=(0,0)$ 到 $x(1)=(1,0)$（停—停，$T=1$ s），最小化 $J=\int_0^1\tfrac12u^2dt$。

- 哈密顿量 $H=\tfrac12u^2+\lambda_1x_2+\lambda_2u$；
- 协态方程：$\dot\lambda_1=-\partial H/\partial x_1=0\Rightarrow\lambda_1=c_1$；$\dot\lambda_2=-\partial H/\partial x_2=-\lambda_1\Rightarrow\lambda_2=-c_1t+c_2$；
- 最优性：$\partial H/\partial u=u+\lambda_2=0\Rightarrow u=c_1t-c_2$，**控制是时间的线性函数**；
- 积分：$x_2=\tfrac12c_1t^2-c_2t$，$x_1=\tfrac16c_1t^3-\tfrac12c_2t^2$；
- 终端条件 $x_2(1)=0\Rightarrow c_2=\tfrac12c_1$；$x_1(1)=1\Rightarrow\tfrac16c_1-\tfrac14c_1=1\Rightarrow c_1=-12$，$c_2=-6$。

于是 $u^\ast(t)=6-12t$，$x_1(t)=3t^2-2t^3$，目标函数 $J=\int_0^1\tfrac12u^2dt=6$，即 $\int_0^1u^2dt=12$（Python 数值积分与后文配点结果 $12.18$ 一致）。这条位置曲线与 [机械臂轨迹生成](../03_运动学与坐标变换/08_机械臂轨迹生成_多项式_梯形速度_S曲线.md) 里的五次多项式 $10\tau^3-15\tau^4+6\tau^5$ **不同**：这里的目标是最小能量（$\int u^2$ 最小），五次多项式是最小 jerk（$\int\dddot x^2$ 最小）。

> **提示**：这个例子说明最优控制的一个典型特征：**最优解的形状由代价函数决定**。换成 $\int u^2$、$\int\dddot x^2$ 或时间，得到的轨迹完全不同。

### 4.3 与 LQR 的联系

对线性系统 $\dot x=Ax+Bu$、代价 $\tfrac12\int(x^\top Qx+u^\top Ru)dt$：$u=-R^{-1}B^\top\lambda$；若设 $\lambda=Px$，代入协态方程得到 Riccati 方程 $-\dot P=A^\top P+PA-PBR^{-1}B^\top P+Q$。所以 **LQR 的 $P$ 就是协态与状态之间的线性关系**，这一点把变分法和 [LQR 最优控制](./04_LQR最优控制.md) 联系起来。

### 4.4 局限

Pontryagin 给出必要条件，导出的是一个两点边值问题（状态正向、协态反向），难以直接数值求解，且对状态不等式约束处理复杂。因此工程上更常用动态规划（反向递推）或直接法（把问题离散为大规模 NLP）。

## 5. 动态规划与 Riccati 的来历

### 5.1 Bellman 原理

最优策略的任何后半段也是最优的。定义价值函数（从第 $k$ 步状态 $x$ 出发的最小剩余代价）：

$$V_N(x)=\ell_N(x),\qquad V_k(x)=\min_u\big[\ell(x,u)+V_{k+1}(f(x,u))\big].$$

括号里的量称为 $Q$ 函数：$Q_k(x,u)=\ell(x,u)+V_{k+1}(f(x,u))$。

### 5.2 LQR 的特例

若 $f=Ax+Bu$，$\ell=\tfrac12(x^\top Qx+u^\top Ru)$，$V_{k+1}=\tfrac12x^\top P_{k+1}x$，则 $Q_k=\tfrac12x^\top Qx+\tfrac12u^\top Ru+\tfrac12(Ax+Bu)^\top P_{k+1}(Ax+Bu)$，对 $u$ 求导置零得 $u=-K_kx$，$K_k=(R+B^\top P_{k+1}B)^{-1}B^\top P_{k+1}A$，代回得 $P_k=Q+A^\top P_{k+1}A-A^\top P_{k+1}BK_k$——即离散 Riccati 递推。**价值函数是二次型，最优策略是线性反馈。**

### 5.3 非线性情形：维数灾难

一般非线性系统的价值函数没有闭式，在整个状态空间上存储 $V$ 会遇到维数灾难（状态维数每增加一维，网格数指数增长）。iLQR/DDP 的思路是：**不求全局的 $V$，只在当前轨迹附近用二次函数近似它**——正好用上 LQR 的结构。

## 6. iLQR：推导

### 6.1 总体流程

给定一条初始控制序列 $\bar u_{0:N-1}$：

1. **前向滚动（rollout）**：用 $\bar u$ 积分得标称轨迹 $\bar x_{0:N}$；
2. **线性化与二次化**：在每个 $(\bar x_k,\bar u_k)$ 处计算 $A_k=f_x$、$B_k=f_u$ 以及代价的导数；
3. **反向递推**：从 $k=N-1$ 到 $0$，递推价值函数的二阶近似，得到 $k_k$（前馈）和 $K_k$（反馈）；
4. **前向更新（带线搜索）**：用 $u_k=\bar u_k+\alpha k_k+K_k(x_k-\bar x_k)$ 重新滚动，$\alpha\in(0,1]$ 逐步减小直到代价下降；
5. 重复，直到收敛。

### 6.2 反向递推公式

设扰动 $\delta x=x-\bar x$，$\delta u=u-\bar u$。已知下一步价值函数的二阶近似：梯度 $V_x'$、海森 $V_{xx}'$。把 $Q$ 函数在标称点附近展开到二阶：

$$Q_x=\ell_x+A^\top V_x',\qquad Q_u=\ell_u+B^\top V_x',$$
$$Q_{xx}=\ell_{xx}+A^\top V_{xx}'A,\quad Q_{uu}=\ell_{uu}+B^\top V_{xx}'B,\quad Q_{ux}=\ell_{ux}+B^\top V_{xx}'A.$$

（iLQR 忽略 $V_x'$ 与动力学二阶导数的缩并项，这是它与 DDP 的区别。）最小化关于 $\delta u$ 的二次函数：

$$\delta u^\ast=\underbrace{-Q_{uu}^{-1}Q_u}_{k}+\underbrace{(-Q_{uu}^{-1}Q_{ux})}_{K}\delta x.$$

将 $\delta u^\ast$ 代回，得到当前步的价值函数近似：

$$V_x=Q_x-Q_{ux}^\top Q_{uu}^{-1}Q_u,\qquad V_{xx}=Q_{xx}-Q_{ux}^\top Q_{uu}^{-1}Q_{ux}.$$

终端：$V_x=\partial\ell_N/\partial x$，$V_{xx}=\partial^2\ell_N/\partial x^2$。

**对比 LQR**：若动力学线性、代价二次，$k=0$（Q_u 在最优点为 0），$K$ 就是 LQR 的反馈增益，$V_{xx}$ 就是 Riccati 的 $P$。所以 **iLQR 是"沿着轨迹逐点做时变 LQR"**。

## 7. 手算：标量系统的 iLQR 全过程

### 7.1 问题

标量非线性系统（示例，非项目实测）：

$$x_{k+1}=x_k+0.2\sin x_k+0.2u_k,$$

$N=3$ 步；代价 $\ell=\tfrac12(x^2+0.1u^2)$，终端 $\ell_N=\tfrac12\cdot10x_N^2$；初值 $x_0=1$。系统开环不稳定（$\partial f/\partial x=1+0.2\cos x\approx1.2>1$）。

### 7.2 标称轨迹

初始控制 $\bar u=(0,0,0)$：$x_1=1+0.2\sin1=1.1683$，$x_2=1.3523$，$x_3=1.5476$。代价 $J=\tfrac12(1+1.3649+1.8287)+\tfrac12\cdot10\times2.3950=2.0968+11.975\approx14.07$（Python：$14.0715$）。

### 7.3 第一次反向递推

$f_u=0.2$，$A_k=1+0.2\cos\bar x_k$。终端 $V_x=10\times1.5476=15.4756$，$V_{xx}=10$。

**$k=2$**（$\bar x_2=1.3523$，$A_2=1.0434$）：

- $Q_x=\bar x_2+A_2V_x=1.3523+1.0434\times15.4756=17.499$；
- $Q_u=0.1\bar u_2+B V_x=0+0.2\times15.4756=3.0951$；
- $Q_{uu}=0.1+0.2^2\times10=0.5$；$Q_{ux}=B\,V_{xx}A_2=0.2\times10\times1.0434=2.0867$；$Q_{xx}=1+A_2^2\times10=11.886$；
- $k_2=-Q_u/Q_{uu}=-6.1902$；$K_2=-Q_{ux}/Q_{uu}=-4.1734$；
- 价值函数：$V_x=Q_x-Q_{ux}Q_u/Q_{uu}=17.499-2.0867\times3.0951/0.5=4.582$；$V_{xx}=Q_{xx}-Q_{ux}^2/Q_{uu}=11.886-8.708=3.178$。

**$k=1$**（$A_1=1.0783$）：用 $V_x=4.582$，$V_{xx}=3.178$：$Q_u=0.2\times4.582=0.9163$，$Q_{uu}=0.1+0.04\times3.178=0.2271$，$Q_{ux}=0.2\times3.178\times1.0783=0.6852$ → $k_1=-4.0351$，$K_1=-3.0174$。

**$k=0$**（$A_0=1.1081$）：$k_0=-3.2611$，$K_0=-2.8387$。

（以上数字均由 Python 逐步打印，与手算一致。）

### 7.4 第一次前向更新

$\alpha=1$。$\delta x_0=0$，$u_0=\bar u_0+k_0=-3.2611$，$x_1=1+0.2\sin1+0.2\times(-3.2611)=0.5161$。$u_1=0+k_1+K_1(x_1-\bar x_1)=-4.0351+(-3.0174)(0.5161-1.1683)=-4.0351+1.9680=-2.0671$，$x_2=0.2013$。$u_2=k_2+K_2(0.2013-1.3523)=-6.1902+4.8039=-1.3868$，$x_3=-0.0360$。代价从 $14.07$ 降到 **$1.5015$**，第一步就大幅下降，不需要线搜索折减。

### 7.5 后续迭代

| 迭代 | 总代价 $J$ | 控制序列 $u_{0:2}$ | 终端状态 $x_3$ |
|---|---|---|---|
| 初始 | 14.0715 | $(0,0,0)$ | 1.5476 |
| 1 | 1.5015 | $(-3.261,-2.067,-1.387)$ | $-0.0360$ |
| 2 | 1.4519 | $(-3.244,-1.879,-1.165)$ | $0.0582$ |
| 3 | 1.451906 | $(-3.2436,-1.8783,-1.1656)$ | 0.0583 |
| 4 | 1.451906 | $(-3.2436,-1.8783,-1.1656)$（前馈 $k$ 已降到 $10^{-4}$ 以下） | 0.0583 |

三次迭代收敛（前馈项 $k_k\to0$ 表示已满足最优性）。最优反馈增益 $K=(-3.062,-3.558,-4.777)$（符号约定 $u=\bar u+k+K\delta x$）。对比把系统在原点线性化（$A=1.2$，$B=0.2$）直接做 LQR 得到的增益 $(3.363,3.649,4.800)$：最后一步 $4.777$ 与 $4.800$ 接近，因为终点附近 $\bar x_2\approx0.24$ 离原点近，线性化误差小；起点附近差别大（$3.06$ 对 $3.36$），因为 $x\approx1$ 处非线性明显。**这正是 iLQR 的价值：增益沿轨迹随线性化点变化。**

## 8. DDP、正则化与线搜索

### 8.1 DDP

微分动态规划（DDP）在展开 $Q$ 函数时保留动力学的二阶项：

$$Q_{xx}=\ell_{xx}+A^\top V_{xx}'A+V_x'\cdot f_{xx},\ \ Q_{uu}=\ell_{uu}+B^\top V_{xx}'B+V_x'\cdot f_{uu},\ \ Q_{ux}=\ell_{ux}+B^\top V_{xx}'A+V_x'\cdot f_{ux},$$

其中 $V_x'\cdot f_{xx}$ 表示 $\sum_iV'_{x,i}\,\nabla^2f_i$。DDP 的收敛速度理论上是二次的，iLQR 是超线性（接近）；但 DDP 需要二阶张量，计算量大，实践中 iLQR 在多数机器人问题上已足够，是更常见的选择。

### 8.2 正则化

要求 $Q_{uu}\succ0$ 才能保证 $\delta u^\ast$ 是下降方向。常用 Levenberg–Marquardt 式正则化：用 $Q_{uu}+\mu I$ 代替 $Q_{uu}$，若分解失败（非正定）则增大 $\mu$；成功则逐步减小。$\mu$ 大 ⇒ 步长小而保守，类似梯度下降；$\mu\to0$ ⇒ 牛顿式。

### 8.3 线搜索

前向更新使用 $\alpha\in\{1,0.5,0.25,\dots\}$ 缩小前馈项，接受第一个使代价充分下降的 $\alpha$。动力学非线性强、线性化不准时，$\alpha=1$ 的完整步可能使代价上升，必须折减。

## 9. 打靶法与直接配点

### 9.1 单打靶（single shooting）的问题

只把 $u_{0:N-1}$ 当变量，状态通过滚动动力学得到。问题：对不稳定系统，$x_N$ 对 $u_0$ 的灵敏度随时域指数增长。**手算**：$\dot x=2x+u$（开环不稳定，极点 $+2$），时域 $T=5$ s：初始控制扰动经 $e^{2\times5}=2.2\times10^4$ 倍放大，优化问题条件数极差，微小数值误差就让终端偏差巨大。

### 9.2 多重打靶（multiple shooting）

把时域切成 $M$ 段，每段起点状态也作为变量，段与段之间加"连续性约束"。每段只需要放大 $e^{aT/M}$：$M=5$ 时 $e^{2\times1}=7.4$（Python 复核），条件数大为改善；代价是变量更多、约束更多，但矩阵稀疏。

### 9.3 直接配点（direct collocation）

把状态和控制都放在离散节点上，用数值积分公式把动力学转成等式约束（**缺陷 defect** 为零）。梯形配点：

$$x_{k+1}-x_k-\frac h2\big[f(x_k,u_k)+f(x_{k+1},u_{k+1})\big]=0.$$

Hermite–Simpson 配点精度更高（三阶），需要中点状态。所有变量一起交给 NLP 求解器（如 SQP、内点法）。

### 9.4 手算与数值：双积分器最小能量

用梯形配点把 §4.2 的问题离散为 $N$ 段（节点 $N+1$ 个，变量 $x_1,x_2,u$ 各 $N+1$ 个），目标是梯形积分的 $\int u^2dt$，约束是 $N$ 段的缺陷与边界条件。这是一个等式约束 QP，直接解 KKT 系统（Python）：

| 段数 $N$ | 配点法得到的 $\int u^2dt$ | 与解析值 12 的差 |
|---|---|---|
| 2 | 32.00 | 巨大（太粗，控制只有 3 个值 $8,0,-8$） |
| 4 | 15.06 | 3.06 |
| 8 | 12.72 | 0.72 |
| 16 | 12.18 | 0.18 |

误差随步长近似二次下降（$N$ 翻倍，误差大约降到 $1/4$）。$N=16$ 时最优控制 $\{5.71,5.33,4.57,\dots,-5.71\}$，对称，与解析解 $6-12t$ 吻合（端点 $\pm6$ 与离散值 $\pm5.71$ 的差来自端点处的梯形权重）。

### 9.5 方法对比

| 对比 | iLQR / DDP | 单打靶 | 多重打靶 | 直接配点 |
|---|---|---|---|---|
| 变量 | 控制（动态规划结构） | 控制 | 控制 + 段起点状态 | 状态 + 控制 |
| 动力学满足 | 精确（滚动） | 精确 | 段内精确，段间约束收敛后满足 | 收敛后满足 |
| 约束处理 | 较弱，需增广 | NLP 求解器 | NLP | NLP，自然 |
| 不稳定系统 | 较好（反馈项稳定轨迹） | 差 | 好 | 好 |
| 计算量 | $O(N\cdot n^3)$，结构化 | 小问题快 | 中 | 大但稀疏 |
| 初值依赖 | 对初始控制敏感 | 敏感 | 状态可给猜测 | 状态猜测可帮助收敛 |
| 典型用途 | 在线 MPC、全身控制 | 小问题 | 通用 | 离线规划，复杂约束 |

## 10. 约束的处理

1. **控制盒约束** $u_{\min}\le u\le u_{\max}$：在 iLQR 反向递推里，每步求解一个盒约束 QP（投影牛顿法），得到被约束的 $k,K$；
2. **一般不等式约束** $g(x,u)\le0$：
   - **罚函数/障碍函数**：把 $-\mu\log(-g)$ 加入代价（内点思想）；
   - **增广拉格朗日（ALTRO 等）**：外层更新乘子，内层用 iLQR 解无约束问题；
   - **SQP**：每次迭代线性化约束，解一个 QP 子问题；
3. **状态约束的软化**：加松弛变量并加大罚权重，保证问题总有解（可行性）。
4. **终端约束**：用增广或直接配点中的等式约束处理。

实践提示：约束越多、越“硬”，对初始猜测和求解器鲁棒性要求越高；在线 MPC 中通常把状态约束软化，控制约束保持硬。

## 11. MPC 的求解器与实时迭代

### 11.1 MPC 的流程

每个控制周期：（1）读取当前状态 $x_0$；（2）求解有限时域 $N$ 步的最优控制问题；（3）只执行第一步 $u_0^\ast$；（4）下一周期重复。这就是 [MPC 模型预测控制](./05_MPC模型预测控制.md) 的滚动时域。

### 11.2 计算预算

假设控制周期 $20$ ms（50 Hz），求解器必须在 $<20$ ms 内返回。车辆运动学模型 $n=3$（$x,y,\psi$），$m=2$，$N=20$：iLQR 每次迭代约 $N\times(\text{几个}\ 3\times3\ \text{矩阵运算})$，微秒到毫秒级；稠密 QP 的变量数 $N\times m=40$；稀疏 QP 的变量数 $N(n+m)=100$，结构化稀疏求解器可达毫秒级。需要实测，数值取决于实现与硬件（示例估计，非项目实测）。

### 11.3 工程技巧

1. **热启动（warm start）**：用上一周期的最优解"移位一格"（丢掉第一步，末尾重复）作为本周期初值，使迭代 1–2 次就收敛；
2. **实时迭代（real-time iteration，RTI）**：每个周期只做**一次**SQP/iLQR 迭代（不要求收敛），依靠连续的热启动让解始终在最优附近。这是 Diehl 等人提出的思想（文献细节未核对）；
3. **凝聚（condensing）**：把状态用控制表示后消去，得到只含 $u$ 的小稠密 QP（适合小 $N$）；不凝聚则保持稀疏结构（适合大 $N$）；
4. **固定迭代次数/时间上限**：保证最坏计算时间可控（硬实时），超时则使用上一周期解的延伸；
5. **可行性回退**：QP 不可行时放松软约束，或切换到预先设计的安全控制器（见 P6 安全执行链的思想）。

工具层面，常见的开源求解器与框架包括 OSQP、qpOASES、HPIPM、acados、CasADi/IPOPT 等（此处仅列名称，具体功能与版本未逐一核对）。

### 11.4 稳定性与终端处理

有限时域 MPC 不自动保证稳定。常见保证稳定的做法：（1）终端代价取为无约束 LQR 的 $P$（使有限时域代价等价于无穷时域）；（2）终端约束集取 LQR 的不变集；（3）时域取得足够长。实践中常取"终端代价 = LQR 的 $P$"，既简单又有效。

## 12. 在本项目里怎么用

> 说明：CV 中 P5 的 MPC 是"基于车式运动学模型、在位置/航向/速度/转向约束下求解滚动时域控制，闭合从规划轨迹到车辆运动的回路"；P1、P7 的是 LQR。**iLQR、DDP、直接配点没有出现在 CV 中**，属延伸知识，面试时用作"我理解这些方法与自己用过的 LQR、MPC 的关系"。

- **P5 MPC 轨迹跟踪**：模型是离散化的车辆运动学，代价是跟踪误差 + 控制量 + 控制变化，约束是速度、转向角、转向速度。如果问"你的 MPC 怎么求解"，本章给出思路：转成 QP（线性化）逐周期求解，热启动，固定时间预算；若问"为什么不用 iLQR"：约束多且硬，QP/SQP 对约束更自然（见 [MPC 轨迹跟踪](../../01_项目课程/P5_车式移动机器人自主导航/10_MPC轨迹跟踪.md) 与 [轨迹跟踪：Pure Pursuit、Stanley、MPC](../08_规划与导航/04_轨迹跟踪_PurePursuit_Stanley_MPC.md)）。
- **P1 / P7 LQR**：iLQR 的反向递推在线性系统上退化成 LQR（§6.2）；P7 的增益调度 LQR 可以看作"沿腿长这一参数的一族 LQR"，而 iLQR 是"沿时间的一族 LQR"，两者结构同源（见 [P7 LQR 原理与增益调度](../../01_项目课程/P7_可变腿长轮腿机器人/05_LQR原理与增益调度.md)）。
- **离线规划角度**：机械臂时间最优搬运、倒立摆起摆、跳跃落地可以用直接配点离线求出参考轨迹，再用 LQR/PD 在线跟踪（两层结构），这是学术界和工业界都常见的组合。
- **与第 11 章的联系**：轨迹的时间律可以用 TOPP 或 S 曲线（[机械臂轨迹生成](../03_运动学与坐标变换/08_机械臂轨迹生成_多项式_梯形速度_S曲线.md)）；若要同时优化路径和时间，则进入本章的一般轨迹优化。

## 13. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| iLQR 代价上升/发散 | 线性化点太远，$Q_{uu}$ 非正定 | 加正则化 $\mu$，线搜索折减 $\alpha$ |
| 对初始控制极敏感，停在局部最优 | 非凸问题，初值差 | 用合理的初值（如 LQR 轨迹、启发式），多起点 |
| 单打靶对不稳定系统发散 | 灵敏度指数增长 | 多重打靶或配点；加稳定反馈 |
| 配点解“锯齿”振荡 | 步长太粗、梯形格式在刚性问题上有数值振荡 | 加密，或用 Hermite–Simpson |
| MPC 偶发不可行，输出突变 | 状态约束太硬，扰动后不可行 | 软约束 + 松弛变量；回退控制器 |
| MPC 求解时间不稳定 | 迭代次数不固定 | 固定迭代/时间预算，RTI，热启动 |
| MPC 闭环不稳定 | 时域太短、无终端代价 | 增大 $N$，终端代价取 $P$ |
| 约束处处满足但轨迹抖 | 约束激活集频繁切换 | 加控制变化代价、平滑项 |
| 模型与实机不符导致轨迹不可执行 | 动力学误差、执行器延迟未建模 | 加延迟状态，保守约束，闭环反馈 |

## 14. 面试怎么问、怎么答

**Q1. How does iLQR relate to LQR and MPC?**

> LQR solves the linear-quadratic problem in closed form with a constant gain. iLQR handles nonlinear dynamics by linearising along a nominal trajectory, doing a time-varying LQR backward pass to get a feedforward term and a feedback gain at each step, rolling out forward with a line search, and repeating. MPC is a way of using such an optimiser online: at each control cycle you solve a finite-horizon problem from the current state, apply the first control, and repeat, usually warm-started from the previous solution.

中文：LQR 对线性二次问题给闭式常增益；iLQR 沿标称轨迹线性化，反向做时变 LQR 得到每步前馈与反馈增益，前向带线搜索滚动，迭代；MPC 是在线使用这类优化器的方式：每周期从当前状态解有限时域问题，执行第一步，并用上次解热启动。

**Q2. Shooting versus collocation?**

> In shooting the controls are the only variables and the states come from simulating the dynamics, so dynamics are always satisfied but the sensitivity grows exponentially for unstable systems. Multiple shooting breaks the horizon into segments to limit that. In direct collocation states and controls are both variables and the dynamics become equality constraints at the nodes. It is sparse, handles constraints naturally and tolerates a bad initial guess better, but the problem is larger and needs an NLP solver.

中文：打靶只以控制为变量，状态由仿真得到，动力学始终满足，但不稳定系统灵敏度指数增长；多重打靶分段限制之；配点把状态与控制都当变量，动力学为节点处等式约束，稀疏、处理约束自然、对初值更宽容，但问题更大、需要 NLP 求解器。

**Q3. What is a Hamiltonian and the costate in optimal control?**

> The Hamiltonian is the running cost plus the costate times the dynamics. The costate is the sensitivity of the optimal cost to the state, and it is integrated backward from the terminal cost. The optimal control minimises the Hamiltonian at every instant. For the linear-quadratic case the costate is linear in the state, and that matrix is the Riccati solution.

中文：哈密顿量 = 阶段代价 + 协态 × 动力学；协态是最优代价对状态的灵敏度，从终端代价反向积分；最优控制在每一时刻最小化哈密顿量；线性二次情形协态与状态成线性关系，系数矩阵即 Riccati 解。

**Q4. Why does MPC use a warm start and real-time iteration?**

> The problem changes only slightly between control cycles, so the previous solution, shifted by one step, is already close to the new optimum. Starting there, one or two Newton-type iterations are enough. Real-time iteration goes further and does only one iteration per cycle, which makes the computation time predictable and still tracks the optimum closely. If time runs out I fall back to the shifted previous plan or to a safe controller.

中文：相邻周期问题变化很小，上次解左移一格已接近新最优，迭代 1–2 次即可；实时迭代每周期只做一次迭代，计算时间可预测且紧跟最优；时间不够时退回移位后的上次计划或安全控制器。

**Q5. How do you handle constraints in trajectory optimisation?**

> For simple bounds on the controls I solve a box-constrained quadratic program inside the backward pass. For general inequality constraints I use an SQP or interior-point NLP solver, or an augmented Lagrangian wrapped around iLQR. State constraints are often softened with slack variables so the problem stays feasible after a disturbance, while input limits stay hard because actuators physically enforce them.

中文：控制盒约束在反向递推里解盒约束 QP；一般不等式用 SQP/内点或把 iLQR 嵌入增广拉格朗日；状态约束常用松弛变量软化以免扰动后不可行，输入限幅保持硬约束因为执行器物理上强制。

## 15. 本章要点速记

1. 轨迹优化 = 在动力学与约束下找最优状态—控制轨迹；LQR 是线性二次无约束的闭式解。
2. Pontryagin：$H=\ell+\lambda^\top f$，$\dot\lambda=-H_x$，$u^\ast=\arg\min H$；双积分器最小能量 $u=6-12t$，$J=12$。
3. 动态规划：$V_k(x)=\min_u[\ell+V_{k+1}(f)]$；LQR 的价值函数是二次型，最优策略线性。
4. iLQR 反向：$Q_x=\ell_x+A^\top V_x'$，$Q_{uu}=\ell_{uu}+B^\top V_{xx}'B$，$k=-Q_{uu}^{-1}Q_u$，$K=-Q_{uu}^{-1}Q_{ux}$；前向 $u=\bar u+\alpha k+K\delta x$。
5. iLQR 在线性系统上等价 LQR；沿轨迹得到时变增益。
6. DDP 保留动力学二阶项；正则化 $Q_{uu}+\mu I$；线搜索折减 $\alpha$。
7. 单打靶对不稳定系统灵敏度 $e^{aT}$（$a=2,T=5$ → $2.2\times10^4$）；多重打靶 $e^{aT/M}$。
8. 梯形配点缺陷 $x_{k+1}-x_k-\frac h2(f_k+f_{k+1})=0$，误差随步长二次下降（$N=4,8,16$：$15.06,12.72,12.18$）。
9. 约束：盒约束 QP、增广拉格朗日、SQP/内点；状态约束软化。
10. MPC：热启动、RTI、固定迭代预算、终端代价取 LQR 的 $P$。

## 16. 自测题

1. 双积分器 $\ddot x=u$，时间 $T=2$ s，从 0 到 1 停—停，最小能量解的最大 $\lvert u\rvert$ 是多少？（最优控制仍是 $t$ 的线性函数，由边界条件得 $u(t)=\frac{6\Delta q}{T^2}\big(1-\frac{2t}{T}\big)$，最大值在端点，$6\times1/4=1.5$ m/s²。）
2. iLQR 反向递推里为什么要求 $Q_{uu}\succ0$？（否则 $\delta u^\ast$ 不是最小而是鞍点/最大，更新方向不下降。）
3. 标量例子最后一步 $Q_{uu}=0.5$ 是怎么来的？（$r+B^2V_{xx}=0.1+0.2^2\times10$。）
4. 多重打靶分 10 段，不稳定极点 $+2$、总时域 $T=5$，每段灵敏度放大多少？（$e^{2\times0.5}=2.7$。）
5. RTI 与"解到收敛"的 MPC 相比，优缺点？（计算时间固定、延迟小，但每周期解不完全最优；靠热启动让误差保持很小。）
6. 为什么 MPC 的终端代价常取 LQR 的 $P$？（让有限时域代价近似无穷时域代价，提高闭环稳定性。）

## 17. 相关链接

- 上一章：[力控与阻抗、导纳、混合力位控制](./12_力控与阻抗_导纳_混合力位控制.md)；下一章：[系统辨识与参数整定](./14_系统辨识与参数整定.md)
- 相关基础：[LQR 最优控制](./04_LQR最优控制.md)、[MPC 模型预测控制](./05_MPC模型预测控制.md)、[最优化与二次规划](../01_数学基础/04_最优化与二次规划.md)
- 项目：[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[05_控制理论 README](./README.md)
