# 轨迹跟踪：Pure Pursuit、Stanley 与 MPC

> **一句话定位**：规划给出"该走哪条线"，跟踪控制器负责"方向盘和油门怎么给才能贴着这条线走"。本章讲车式底盘三种常用跟踪方法：几何法 Pure Pursuit、Stanley，以及基于模型的 MPC（P5 CV 的最后一环）。

## 本章学完你能……

- 推导 Pure Pursuit 的曲率公式 $\kappa=2\sin\alpha/L_d$ 和转向角，并手算一例。
- 写出 Stanley 控制律，说明航向项与横向项的作用，手算一例并解释误差收敛。
- 建立误差模型，手算一个 $N=2$ 的 MPC（解析最优 $u_0=-0.107,\ u_1=0.089$），并看到限幅对解的影响。
- 说清横向 / 纵向解耦、参考轨迹生成（最近点、弧长、曲率）和控制延迟补偿。
- 对比三种方法的适用场景，用英文回答面试问题。

## 前置知识

- 自行车模型：[Hybrid A* 与车辆运动学约束](./02_Hybrid_A星与车辆运动学约束.md) 第 2 节
- MPC 理论：[MPC 模型预测控制](../04_控制理论/05_MPC模型预测控制.md)；离散化：[数字控制与实时性](../04_控制理论/08_数字控制与实时性.md)

## 目录

1. 跟踪问题与误差定义
2. Pure Pursuit
3. Stanley
4. MPC 跟踪
5. 横向 / 纵向解耦与速度规划
6. 参考轨迹生成
7. 延迟及补偿
8. 在本项目里怎么用
9. 常见错误
10. 面试问答
11. 速记、自测、链接

---

## 1. 跟踪问题与误差定义

**问题**：参考路径 $\{(x_r,y_r,\theta_r,\kappa_r)\}$，车的状态 $(x,y,\theta,v)$。要求误差趋于 0，同时转向角不超限、不抖。

**符号**：$e_y$ 横向误差（车在路径左侧为正，m）；$e_\psi=\theta-\theta_r$ 航向误差（rad，归一化到 $(-\pi,\pi]$）；$L$ 轴距；$\delta$ 前轮转角；$L_d$ 前视距离。

**误差运动学（小角度）**：以路径为参考，车速 $v$ 时

$$
\dot e_y=v\sin e_\psi\approx v\,e_\psi,\qquad \dot e_\psi=\frac vL\tan\delta-v\kappa_r\approx\frac vL\delta-v\kappa_r
$$

直路上 $\kappa_r=0$，则输入 $\delta$ 先改变 $e_\psi$，再改变 $e_y$——二阶链，这就是"方向盘打了之后位置过一会儿才变"的原因。

---

## 2. Pure Pursuit

### 2.1 思想

在路径上取距后轴中心 $L_d$ 远的**目标点**，让车沿一段过后轴中心、与车头相切的圆弧开向它。

### 2.2 推导

目标点在车身系坐标 $(x_l,y_l)$，$\alpha$ 为车头方向与"后轴到目标点连线"的夹角，$x_l^2+y_l^2=L_d^2$，$y_l=L_d\sin\alpha$。圆弧与车头相切于原点，圆心在车身 y 轴上 $(0,R)$，目标点在圆上：

$$
x_l^2+(y_l-R)^2=R^2\ \Rightarrow\ L_d^2-2y_lR=0\ \Rightarrow\ R=\frac{L_d^2}{2y_l}=\frac{L_d}{2\sin\alpha}
$$

$$
\boxed{\kappa=\frac{2\sin\alpha}{L_d},\qquad \delta=\arctan(L\kappa)=\arctan\frac{2L\sin\alpha}{L_d}}
$$

### 2.3 手算（教学数值）

$L=0.5$ m，$L_d=1.0$ m，目标点方位 $\alpha=30°$（即车身系 $(0.866,\,0.5)$）：
$\kappa=2\times0.5/1=1.0$ 1/m（$R=1$ m）；$\delta=\arctan(0.5\times1.0)=26.57°$。若 $\delta_{max}=30°$，未饱和。若 $\alpha=45°$：$\kappa=1.414$，$\delta=\arctan0.707=35.3°>30°$，需限幅为 30°（此时实际曲率 $1.155$，车会"切内弯"）。

### 2.4 前视距离的取舍

常用 $L_d=k_vv+L_0$（并限幅）。$L_d$ 小：贴路径紧但易振荡、对噪声敏感；$L_d$ 大：平滑但转弯**切内角**、过弯偏内。
性质：只用几何，不含车辆动力学；低速表现好，高速需增大 $L_d$。

---

## 3. Stanley

### 3.1 控制律

以**前轴中心**为参考点（斯坦福 DARPA 车 Stanley 采用，Hoffmann 等 2007，凭记忆引用，未核对）：

$$
\boxed{\delta=\underbrace{e_\psi}_{\text{航向对齐}}+\underbrace{\arctan\frac{k\,e_{fa}}{v+k_s}}_{\text{横向收敛}}}
$$

$e_{fa}$：前轴到最近路径点的横向误差；$k$ 增益；$k_s$ 低速软化常数（避免 $v\to0$ 时除零）。符号约定：误差定义为使 $\delta$ 把车拉回路径的方向（与上面 $e_y$ 差一个负号时，公式取 $-$），实现时要统一。

### 3.2 为什么收敛

只看横向项，$\dot e\approx-v\sin\!\big(\arctan\frac{ke}{v}\big)\approx-ke$，即 $e(t)=e_0e^{-kt}$，**指数收敛，速率与车速无关**；$e$ 大时 $\arctan$ 饱和，转向角不会无限增大。

### 3.3 手算

$k=1.5$，$v=1.0$，$k_s=0$，$e=0.3$ m，$e_\psi=10°$：
横向项 $\arctan(1.5\times0.3/1.0)=\arctan0.45=24.2°$；$\delta=10°+24.2°=34.2°$，超 30°，限幅到 30°。
若已平行路径（$e_\psi=0$）：$\delta=24.2°$，随后 $e$ 变小，$\delta$ 减小。

### 3.4 与 Pure Pursuit 对比

| | Pure Pursuit | Stanley |
|---|---|---|
| 参考点 | 后轴 | 前轴 |
| 信息 | 前视点 | 最近点 + 航向误差 |
| 弯道 | 切内角 | 直路表现好，弯道需曲率前馈 |
| 调参 | $L_d$ | $k,k_s$ |

---

## 4. MPC 跟踪

### 4.1 思想

每个周期：用模型预测未来 $N$ 步，选一串控制 $u_0\ldots u_{N-1}$ 使跟踪误差与控制量的加权和最小，且满足约束；**只执行 $u_0$**，下周期重来（滚动时域）。理论见[MPC 章](../04_控制理论/05_MPC模型预测控制.md)。CV：基于车式运动学模型，在位置、航向、速度、转向约束下滚动求解。

### 4.2 线性误差模型

状态 $x=[e_y,e_\psi]^\top$，输入 $u=\delta$，直线参考，欧拉离散（$\Delta t$）：

$$
x_{k+1}=Ax_k+Bu_k,\quad A=\begin{bmatrix}1&v\Delta t\\0&1\end{bmatrix},\quad B=\begin{bmatrix}0\\v\Delta t/L\end{bmatrix}
$$

代价 $J=\sum_{k=1}^{N}x_k^\top Qx_k+\sum_{k=0}^{N-1}Ru_k^2$，约束 $|u_k|\le\delta_{max}$（还可加转向速率约束 $|u_k-u_{k-1}|\le\Delta\delta_{max}$）。弯道时需围绕参考轨迹线性化（时变 $A_k,B_k$），常含曲率前馈。

### 4.3 手算（$N=2$）

$v=1$，$L=0.5$，$\Delta t=0.5$：$A=\begin{bmatrix}1&0.5\\0&1\end{bmatrix}$，$B=\begin{bmatrix}0\\1\end{bmatrix}$。$Q=\mathrm{diag}(1,0.5)$，$R=0.1$，初始 $x_0=[0.2,\,0]^\top$（在路径一侧 0.2 m，航向对齐）。

预测：$x_1=[0.2,\ u_0]^\top$；$x_2=[0.2+0.5u_0,\ u_0+u_1]^\top$。代价：

$$
J=0.04+0.5u_0^2+(0.2+0.5u_0)^2+0.5(u_0+u_1)^2+0.1(u_0^2+u_1^2)
=0.08+0.2u_0+1.35u_0^2+u_0u_1+0.6u_1^2
$$

令梯度为零：$2.7u_0+u_1+0.2=0$，$u_0+1.2u_1=0$。得 $u_1=-u_0/1.2$，$(2.7-0.833)u_0=-0.2$，

$$
u_0^*=-0.1071\ \text{rad}\ (-6.1°),\qquad u_1^*=0.0893\ \text{rad}
$$

$J^*=0.0693$（不控制时 $J=0.08$）。结果：$x_1=[0.2,-0.107]$，$x_2=[0.146,-0.018]$：先朝路径方向偏一点，横向误差减小，航向被拉回——**先"打向路径"再"回正"**，解的结构是预测带来的。
**限幅**：若 $|u|\le0.1$ rad，则 $u_0=-0.1$，$u_1=0.083$，$J=0.0693$（几乎不变）；约束使 $u_0$ 饱和，$u_1$ 自动重新分配——这正是 MPC 对 PID 的优势：约束直接进入优化。
只执行 $u_0=-0.107$，下一周期用新状态重算。

### 4.4 实用要点

求解器 QP（OSQP 等）；$N\Delta t$ 要覆盖足够前视（如 1~2 s）；$Q,R$ 与转向速率惩罚决定平滑；需在控制周期内求完（含热启动）；弯道用曲率前馈 $\delta_{ff}=\arctan(L\kappa_r)$ 减轻 MPC 负担。

---

## 5. 横向 / 纵向解耦与速度规划

- **横向**：转向角控制，跟踪路径形状（上面三种）。
- **纵向**：速度控制（PID 或 MPC），跟踪速度曲线。
- **解耦的前提**：速度变化慢于横向动态；横向模型把 $v$ 当参数。耦合点：速度越大，同样转向角下横向加速度越大。
- **曲率限速**：$v\le\sqrt{a_{lat,max}/|\kappa|}$。例：$a_{lat,max}=1\ \text{m/s}^2$，$\kappa=1.0$ → $v\le1.0$ m/s；$\kappa=0.25$ → $2.0$ m/s。再叠加加减速限制和障碍距离限速，得到速度曲线。
- 联合 MPC 可同时优化 $v,\delta$，但计算更重。

---

## 6. 参考轨迹生成

1. **最近点搜索**：找路径上距车最近的点（限制搜索窗口、保证单调前进，避免回环路径时跳到另一段）。
2. **弧长参数化**：用累计弧长 $s_i$ 作参数，便于按距离取前视点或 MPC 预测点。
3. **插值**：位置线性插值，航向取切线方向 $\theta_r=\operatorname{atan2}(\Delta y,\Delta x)$。
4. **曲率**：离散曲率 $\kappa_i\approx\Delta\phi/\Delta s$，需先平滑（原始点噪声会放大成曲率尖峰）。
5. **误差计算**：横向误差 $e_y=-(x-x_r)\sin\theta_r+(y-y_r)\cos\theta_r$（车在左侧为正）。
手算：最近点 $(2,\,1)$，$\theta_r=0$，车在 $(2.1,\,1.3)$：$e_y=-(0.1)\cdot0+(0.3)\cdot1=0.3$ m（左侧）。
6. **倒车段**：航向参考与行驶方向反向，前视点取"身后"，符号需单独处理（与里程计符号问题同类，见[里程计章](../05_状态估计/04_里程计与传感器融合.md)）。

---

## 7. 延迟及补偿

执行链有延迟 $\tau$（计算 + 通信 + 转向机构）。$v=1$ m/s、$\tau=0.2$ s，则车在指令生效前已走 $0.2$ m，转向指令按"旧状态"算会过量——表现为振荡。
**补偿**：用模型把当前状态向前预测 $\tau$：$x_p=x+v\cos\theta\,\tau$，$y_p=y+v\sin\theta\,\tau$，$\theta_p=\theta+\frac vL\tan\delta\,\tau$；以 $x_p$ 作为控制起点。MPC 中可把 $\tau/\Delta t$ 步的历史输入放入状态（增广），或预测起点后移。

---

## 8. 在本项目里怎么用

CV：*MPC trajectory tracking: based on a car-like kinematic model, solved receding-horizon control under position, heading, speed and steering constraints, closing the loop from planned trajectory to vehicle motion on the real vehicle.* 对应：自行车模型（第 1、4 节）、位置/航向误差为状态（第 4.2 节）、速度和转向约束（第 4.1 节）、参考来自 TEB 的位姿与曲率（[TEB 章](./03_代价地图_局部规划_TEB.md) 第 9 节）。Pure Pursuit / Stanley 是对比基线与调试备份。见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **【待你补全】** 预测时域 $N$、步长、权重、求解器、控制频率、约束数值、与 Pure Pursuit 的对比结果、延迟的处理方式。CV 无这些数字。

## 9. 常见错误

| 现象 | 原因 |
|---|---|
| 直路蛇行 | $L_d$ 太小 / $k$ 太大 / 延迟未补偿 |
| 弯道切内角 | $L_d$ 太大；无曲率前馈 |
| 低速抖动 | Stanley 的 $k_s$ 为 0；横向误差噪声 |
| 转向饱和后发散 | 无限幅意识；MPC 缺转向速率约束 |
| 路径末端振荡 | 最近点在终点附近乱跳；需到达判据与减速 |
| MPC 超时 | $N$ 过大；未热启动 |
| 倒车跟踪反向 | 航向 / 速度符号错 |

## 10. 面试问答

**Q1. Derive pure pursuit.**
> A: I pick a lookahead point at distance L_d on the path and require a circular arc through the rear axle that is tangent to the heading and passes through that point. Geometry gives curvature 2 sin alpha over L_d, and the steering angle is arctan of wheelbase times curvature. A larger L_d is smoother but cuts corners.

**Q2. Why MPC rather than Pure Pursuit?**
> A: MPC uses a model to look ahead and handles constraints explicitly, for example steering angle and rate limits and speed limits, and it can use reference curvature. Pure pursuit is simple and robust but purely geometric, so it cannot enforce constraints. In P5 I used MPC for the real vehicle (the exact weights 【待补全】).

**Q3. What does the Stanley controller do?**
> A: It adds a heading-error term and a cross-track term, arctan of k times error over speed, at the front axle. The cross-track error decays exponentially, and the arctan keeps the command bounded.

**Q4. How do you handle control latency?**
> A: I predict the state forward by the latency using the kinematic model and compute the command from the predicted state, or I augment the MPC state with the commands still in the pipeline.

## 11. 速记、自测、链接

**速记**：$\kappa=2\sin\alpha/L_d$；$\delta=\arctan(L\kappa)$；Stanley $\delta=e_\psi+\arctan(ke/(v+k_s))$；横向误差指数收敛 $e^{-kt}$；MPC 只执行第一步；约束进优化；曲率限速 $\sqrt{a_{lat}/\kappa}$；延迟用模型前推。

**自测**：
1. $L=0.4,L_d=0.8,\alpha=20°$，求 $\delta$。（$\kappa=2\sin20°/0.8=0.855$；$\delta=\arctan0.342=18.9°$。）
2. Stanley $k=2,v=2,e=0.5,e_\psi=0$，$\delta$？（$\arctan0.5=26.6°$。）
3. $a_{lat,max}=2$，$\kappa=0.5$，最高速度？（$\sqrt4=2$ m/s。）
4. 为何 MPC 只执行第一个控制量？（模型和环境有误差，反馈重算比开环执行更稳。）
5. 车速 2 m/s、延迟 0.15 s，车在延迟内走多远？（0.3 m。）

**链接**：上一章 [代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)；下一章 [导航系统架构](./05_导航系统架构_move_base与Nav2.md)；[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)；[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)。
