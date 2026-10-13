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
- MPC 理论：[MPC 模型预测控制](../05_控制理论/05_MPC模型预测控制.md)；离散化：[数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)

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

**误差模型从哪里来（推导）**：后轴自行车模型 $\dot x=v\cos\theta,\ \dot y=v\sin\theta,\ \dot\theta=\frac vL\tan\delta$。参考路径上最近点的切线角 $\theta_r$、曲率 $\kappa_r$，沿路径前进的速率 $\dot s=v\cos e_\psi/(1-\kappa_re_y)$。

1. 横向误差是车相对路径法线方向的位置分量，其变化率是车速在法线方向的投影：$\dot e_y=v\sin e_\psi$；
2. 航向误差 $e_\psi=\theta-\theta_r$ 的变化率 = 车头转速减去参考切线转速：$\dot e_\psi=\frac vL\tan\delta-\kappa_r\dot s\approx\frac vL\tan\delta-v\kappa_r$；
3. 小角度 $\sin e_\psi\approx e_\psi$，$\tan\delta\approx\delta$，得到本节的线性形式。

**数值感受（教学数值）**：$v=1$ m/s，$L=0.5$ m，$\delta=0.1$ rad：$\dot e_\psi=2\times0.1=0.2$ rad/s。持续 1 s 后 $e_\psi=0.2$ rad，$e_y$ 约增加 $\int v e_\psi\,dt=0.1$ m。即 1 s 的 0.1 rad 转向只带来 10 cm 的位置变化，所以横向控制必须"提前"。

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

**随速度变化的前视距离（手算，教学数值）**：$L_d=k_vv+L_0$，$k_v=0.8$ s，$L_0=0.3$ m，上下限 $[0.6,\,3.0]$ m。$v=0.5$：$0.7$ m；$v=1.0$：$1.1$ m；$v=2.0$：$1.9$ m；$v=4$：$3.5\to$ 限幅为 3.0 m。取 $L_d\propto v$ 使 2.6 节的 $\omega_n=\sqrt2v/L_d\approx\sqrt2/k_v=1.77$ rad/s 基本不随速度变化，这就是"前视随速度增长"的理论依据（上下限处除外）。

### 2.5 怎么在折线路径上找前视点（圆与线段求交）

实现中路径是点列，前视点 = 以后轴中心 $c$ 为圆心、$L_d$ 为半径的圆与路径折线的**前方**交点。对线段 $p(t)=p_0+t\,d$，$d=p_1-p_0$，$t\in[0,1]$，令 $f=p_0-c$，解 $\|f+td\|^2=L_d^2$：

$$
a=d\cdot d,\quad b=2f\cdot d,\quad c_q=f\cdot f-L_d^2,\quad t=\frac{-b\pm\sqrt{b^2-4ac_q}}{2a}
$$

取 $t\in[0,1]$ 且更靠前的根。**手算（教学数值）**：路径是直线 $y=1$ 上从 $(0,1)$ 到 $(4,1)$；后轴中心 $c=(0,0.4)$，车头朝 $+x$（$\theta=0$）；$L_d=1$，$L=0.5$ m。

- $d=(4,0)$，$f=(0,0.6)$；$a=16$，$b=0$，$c_q=0.36-1=-0.64$；判别式 $=0+4\times16\times0.64=40.96$，$\sqrt{\;}=6.4$；
- $t=\pm6.4/32=\pm0.2$，取 $t=0.2$：前视点 $(0.8,\,1.0)$；
- 车身系坐标 $(x_l,y_l)=(0.8,\,0.6)$，$\alpha=\arctan(0.6/0.8)=36.87°$（正好 3-4-5 三角形，$\sin\alpha=0.6$）；
- $\kappa=2\times0.6/1=1.2$ 1/m，$\delta=\arctan(0.5\times1.2)=30.96°$。

若车头不是朝 $+x$ 而是 $\theta=10°$：$\alpha=36.87°-10°=26.87°$，$\kappa=2\sin26.87°=0.904$，$\delta=\arctan(0.452)=24.3°$。**必须用"目标方位角减车头朝向"**并归一化到 $(-\pi,\pi]$，这是最常见的符号 / 归一化错误点。

没有交点时（车离路径太远，圆与路径不相交）常见处理：取路径上离车最近点之后、距离最接近 $L_d$ 的点；到终点附近前视点超出路径末端时，取终点并随距离缩小速度。

### 2.6 小角度下的误差动力学：Pure Pursuit 为什么稳定

直线参考，设横向误差 $e$（车在路径左侧为正）、航向误差 $\psi=\theta-\theta_r$（小角度）。后轴运动学：

$$
\dot e=v\psi,\qquad \dot\psi=v\kappa
$$

前视点在车身系里的横向分量 $y_l\approx -e-L_d\psi$（路径在车的右边就要向右转），所以 $\sin\alpha\approx y_l/L_d$，$\kappa=2y_l/L_d^2$，代入：

$$
\dot\psi=-\frac{2v}{L_d^2}\,(e+L_d\psi)\ \Rightarrow\ \ddot e=v\dot\psi=-\frac{2v}{L_d}\,\dot e-\frac{2v^2}{L_d^2}\,e
$$

标准二阶系统 $\ddot e+2\zeta\omega_n\dot e+\omega_n^2e=0$，对照得：

$$
\omega_n=\frac{\sqrt2\,v}{L_d},\qquad \zeta=\frac{2v/L_d}{2\omega_n}=\frac{1}{\sqrt2}\approx0.707
$$

**两个重要结论**：(1) 阻尼比与 $v$、$L_d$ 无关，恒为 0.707，所以理想模型下不振荡超调很小；(2) 响应快慢由 $v/L_d$ 决定，$L_d=k_vv$ 时 $\omega_n=\sqrt2/k_v$ 与速度无关。**手算**：$v=1$，$L_d=1$：$\omega_n=1.414$ rad/s，极点 $s=-1\pm j1$，4% 调节时间约 $4/(\zeta\omega_n)=4$ s。$L_d$ 增大到 2 m，$\omega_n$ 减半，调节时间变成 8 s——"平滑但慢、过弯切内角"的定量表现。

实际车会振荡是因为：转向有延迟与一阶滞后（模型里没有）、$L_d$ 取得太小使 $\omega_n$ 接近转向机构带宽、前视点抖动。这与第 7 节延迟分析呼应。

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

### 3.5 推导：为什么是 $\arctan(ke/v)$

**符号约定（本节统一）**：路径切线方向 $\theta_r$，车头朝向 $\theta$；$\psi_e=\theta_r-\theta$（车需要向左转正）；$e_s$ 为前轴中心到路径的横向距离，**路径在车的左边为正**（需要向左转正）。这样两项同号表示"都要向左"，控制律形式是 $\delta=\psi_e+\arctan(ke_s/(v+k_s))$，与上面的公式一致。

**两个设计目标**：
1. 前轮方向与路径切线平行：$\delta=\psi_e$（把航向误差抵消）；
2. 前轴到路径的距离减小：再叠加一个依赖 $e_s$ 的角度。

**前轴速度分解**：前轮方向（相对地面）为 $\theta+\delta$，前轴速度大小记为 $v$。路径朝向 $\theta_r$ 与它的夹角为 $\theta_r-\theta-\delta=\psi_e-\delta$，朝着路径（左侧）的速度分量就是横向误差的减小率：

$$
\dot e_s=v\sin(\psi_e-\delta)
$$

代入控制律 $\delta=\psi_e+\beta$，$\beta=\arctan\dfrac{ke_s}{v+k_s}$：

$$
\dot e_s=-v\sin\beta,\qquad \sin\arctan z=\frac{z}{\sqrt{1+z^2}}\ \Rightarrow\ \dot e_s=-\frac{v}{v+k_s}\cdot\frac{ke_s}{\sqrt{1+\left(\frac{ke_s}{v+k_s}\right)^2}}
$$

**结论**：对任意 $e_s\ne0$，$\dot e_s$ 与 $e_s$ 异号（取 $V=e_s^2$，$\dot V<0$），所以全局渐近稳定。$k_s=0$ 且 $e_s$ 小时 $\dot e_s\approx-ke_s$，指数收敛，时间常数 $1/k$，**与车速无关**；$e_s$ 很大时 $\dot e_s\to-v$ 饱和，对应 $\beta\to90°$，车几乎垂直冲向路径，所以实际中 $\delta$ 必须再限幅。$k_s>0$ 时有效收敛速率减小为 $kv/(v+k_s)$，换来低速时不过激。

**数值验证（教学数值，Euler 积分步长 1 ms，$k=1.5$，$v=1$，$k_s=0$，$e_s(0)=0.3$ m，假设航向已对齐）**：

| 时间 | $e_s$（数值积分） | 纯指数 $0.3e^{-1.5t}$ |
|---|---|---|
| 0.5 s | 0.147 | 0.142 |
| 1 s | 0.070 | 0.067 |
| 2 s | 0.016 | 0.015 |
| 3 s | 0.0035 | 0.0033 |

非线性项使收敛略慢于纯指数，趋势一致。**含 $k_s$ 的手算**：$k=1.5$，$v=1$，$k_s=0.5$，$e_s=0.3$：$\arctan(1.5\times0.3/1.5)=\arctan0.3=16.7°$，比 $k_s=0$ 时的 $24.2°$ 小，即低速软化。

**曲率前馈**：弯道上若只用上式，稳态会有横向偏差（因为 $\dot\psi_e$ 需要持续的转向角来匹配路径曲率）。常见做法在 $\delta$ 上加 $\delta_{ff}=\arctan(L\kappa_r)$，使稳态航向误差为 0。例如 $L=0.5$，$\kappa_r=1$：$\delta_{ff}=26.6°$。

**Stanley 与 Pure Pursuit 的本质差别**：Pure Pursuit 通过"看远处的点"隐含地把航向和横向混在一起（见 2.6 节，恒定 $\zeta=0.707$）；Stanley 把两项分开、各有可调增益，因此在低速、大曲率弯道可以更精细，但对航向噪声更敏感（航向误差直接进入 $\delta$）。

---

## 4. MPC 跟踪

### 4.1 思想

每个周期：用模型预测未来 $N$ 步，选一串控制 $u_0\ldots u_{N-1}$ 使跟踪误差与控制量的加权和最小，且满足约束；**只执行 $u_0$**，下周期重来（滚动时域）。理论见[MPC 章](../05_控制理论/05_MPC模型预测控制.md)。CV：基于车式运动学模型，在位置、航向、速度、转向约束下滚动求解。

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

### 4.5 从预测模型到标准 QP（以 4.3 的例子逐项构造）

**第一步：堆叠预测方程**。令 $X=[x_1;x_2]$，$U=[u_0;u_1]$，则

$$
X=\Phi x_0+\Gamma U,\quad \Phi=\begin{bmatrix}A\\A^2\end{bmatrix},\quad \Gamma=\begin{bmatrix}B&0\\AB&B\end{bmatrix}
$$

代入数值：$A=\begin{bmatrix}1&0.5\\0&1\end{bmatrix}$，$B=\begin{bmatrix}0\\1\end{bmatrix}$，$AB=\begin{bmatrix}0.5\\1\end{bmatrix}$，$A^2=\begin{bmatrix}1&1\\0&1\end{bmatrix}$：

$$
\Phi=\begin{bmatrix}1&0.5\\0&1\\1&1\\0&1\end{bmatrix},\quad
\Gamma=\begin{bmatrix}0&0\\1&0\\0.5&0\\1&1\end{bmatrix}
$$

验证：$x_2=A^2x_0+ABu_0+Bu_1=[0.2,0]^\top+[0.5u_0,\ u_0]^\top+[0,\ u_1]^\top=[0.2+0.5u_0,\ u_0+u_1]^\top$，与 4.3 完全一致。

**第二步：代价写成二次型**。$\bar Q=\mathrm{diag}(Q,Q)$，$\bar R=R\,I$。$J=X^\top\bar QX+U^\top\bar RU$，代入 $X$ 并去掉与 $U$ 无关的常数：

$$
J=\tfrac12U^\top HU+f^\top U+\text{const},\qquad H=2(\Gamma^\top\bar Q\Gamma+\bar R),\quad f=2\Gamma^\top\bar Q\Phi x_0
$$

数值：$\Gamma^\top\bar Q\Gamma$ 的 (1,1) 项 $=1\cdot0^2+0.5\cdot1^2+1\cdot0.5^2+0.5\cdot1^2=1.25$；(1,2) 项 $=0.5\cdot1\cdot1=0.5$；(2,2) 项 $=0.5\cdot1^2=0.5$。所以

$$
H=2\begin{bmatrix}1.25+0.1&0.5\\0.5&0.5+0.1\end{bmatrix}=\begin{bmatrix}2.7&1\\1&1.2\end{bmatrix},\qquad f=\begin{bmatrix}0.2\\0\end{bmatrix}
$$

与 4.3 的梯度方程 $2.7u_0+u_1+0.2=0$、$u_0+1.2u_1=0$ 一致（梯度 $=HU+f$）。$H$ 的特征值均为正（行列式 $2.7\times1.2-1=2.24>0$），所以是严格凸 QP，有唯一解。

**第三步：约束写成 $G\,U\le h$**。幅值约束 $|u_k|\le\delta_{max}$ 写成 $\begin{bmatrix}I\\-I\end{bmatrix}U\le\delta_{max}\mathbf1$；转向速率约束 $|u_k-u_{k-1}|\le\Delta\delta_{max}$ 需要上一周期实际输出 $u_{-1}$：

$$
u_0-u_{-1}\le\Delta,\ \ -(u_0-u_{-1})\le\Delta,\ \ u_1-u_0\le\Delta,\ \ -(u_1-u_0)\le\Delta
$$

**第四步：带速率约束的一步手算**。取 $u_{-1}=0.02$ rad，$\Delta=0.08$ rad/步（教学数值）。约束：$-0.06\le u_0\le0.10$，$|u_1-u_0|\le0.08$。无约束解 $(-0.107,\,0.089)$ 违反两条：$u_0<-0.06$，且 $u_1-u_0=0.196>0.08$。猜测两条都取等号：$u_0=-0.06$，$u_1=u_0+0.08=0.02$。用 KKT 验证：梯度 $HU+f=[2.7(-0.06)+0.02+0.2,\ -0.06+1.2\times0.02]=[0.058,\ -0.036]$；约束 $c_1=-u_0-0.06\le0$ 的梯度 $(-1,0)$，$c_2=u_1-u_0-0.08\le0$ 的梯度 $(-1,1)$。平稳条件 $\nabla J+\lambda_1(-1,0)+\lambda_2(-1,1)=0$：第二个分量 $-0.036+\lambda_2=0\Rightarrow\lambda_2=0.036\ge0$；第一个分量 $0.058-\lambda_1-\lambda_2=0\Rightarrow\lambda_1=0.022\ge0$。乘子均非负，满足 KKT，故即最优。代价 $J=0.0719$，比无约束的 $0.0693$ 只高 3.8%，但**第一步实际下发 $-0.06$ 而不是 $-0.107$**，方向盘不会突变——这就是速率约束的平顺作用。（该结论也用网格穷举复核，最优为同一点。）

### 4.6 弯道：时变线性化与曲率前馈

直线参考时 $A,B$ 是常数；弯道上要围绕参考 $(\kappa_r,\delta_{ff})$ 线性化。误差 $\dot e_\psi=\frac vL\tan\delta-v\kappa_r$。令 $\delta=\delta_{ff}+u'$，$\delta_{ff}=\arctan(L\kappa_r)$，对 $\tan$ 在 $\delta_{ff}$ 处一阶展开：

$$
\tan\delta\approx\tan\delta_{ff}+\sec^2\delta_{ff}\,u'=L\kappa_r+(1+L^2\kappa_r^2)\,u'
$$

于是 $\dot e_\psi=\dfrac{v(1+L^2\kappa_r^2)}{L}u'$，前馈抵消了 $v\kappa_r$ 项。离散后 $B_k=\begin{bmatrix}0\\ v\Delta t(1+L^2\kappa_{r,k}^2)/L\end{bmatrix}$。**手算**：$L=0.5$，$\kappa_r=1$，$v=1$，$\Delta t=0.5$：$1+0.25=1.25$，$B_{22}=0.5\times1.25/0.5=1.25$，直线时是 1.0。弯越急，同样的转向增量产生更大的航向变化率（$\sec^2$ 项）。MPC 优化的变量是 $u'$（相对前馈的修正量），约束要改写为 $|\delta_{ff,k}+u'_k|\le\delta_{max}$。

### 4.7 本项目的 MPC 配置（设计取值，量级，调参时按实机微调）

| 项 | 本项目通常取 | 说明 |
|---|---|---|
| 状态 / 输入 | 状态为位置、航向（及速度）相对参考的误差；输入为转向角（及加速度） | 与 CV 的 "position, heading, speed and steering constraints" 对应 |
| 步长 $\Delta t$ | 0.1 s（可在 0.05–0.1 s 之间） | 与 TEB 输出的轨迹时间分辨率同量级，参考点好对齐 |
| 预测时域 $N$ | 10–20，即 $N\Delta t\approx1$–$2$ s | 要覆盖制动距离与弯道前视；再长对模型误差敏感 |
| 权重 | 横向误差与航向误差权重大于输入权重；另有输入变化率惩罚 $R_d$；权重用"可接受误差的平方倒数"起步（见 P5 的 MPC 章） | 先让直线稳，再调弯道 |
| 求解器 | 凸 QP 求解器（OSQP 类），热启动 | 决策变量 20–40 个，求解毫秒量级 |
| 控制频率 | 10–20 Hz（设计取值） | 求解时间需明显小于周期 |
| 约束数值 | 转向角限幅取机械极限的 90% 左右留余量；转向速率限幅按舵机实测速度的一部分；速度上下限包含倒车 | 数值以车辆标定为准，原则是**软件限幅比硬件极限更保守** |
| 延迟处理 | 缓存已下发但未生效的命令，用模型前推状态后再解 MPC，参考点同时从 $t+\tau$ 起取 | 见第 7 节与 4.8 节 |
| 与 Pure Pursuit 的对比关注点 | 弯道切内角量（偏差随 $L_d$ 变化）、转向平顺性（转角速率峰值）、约束触发时的行为 | 对比的是机理差异：Pure Pursuit 无约束、无曲率感知；MPC 有 |

这套配置回答了面试里"为什么用 MPC、参数大概是多少"：重点是**每一项为什么这样取**，而不是背一个数。Pure Pursuit / Stanley 在本项目中作为对比基线和调试备份。

### 4.8 MPC 与延迟的结合：状态增广（手算维度）

若控制命令有 $d=\tau/\Delta t$ 步延迟（例：$\tau=0.2$ s，$\Delta t=0.1$ s，$d=2$），则 $x_{k+1}=Ax_k+Bu_{k-d}$。增广状态 $z_k=[x_k;\ u_{k-2};\ u_{k-1}]$（2+1+1=4 维）：

$$
z_{k+1}=\begin{bmatrix}A&B&0\\0&0&1\\0&0&0\end{bmatrix}z_k+\begin{bmatrix}0\\0\\1\end{bmatrix}u_k
$$

第 2、3 块是"移位寄存器"：$u_{k-1}$ 变成下一步的 $u_{k-2}$，当前 $u_k$ 进入队尾。初始增广状态里的 $u_{-2},u_{-1}$ 就是已发出但未生效的命令。这样 QP 的结构不变，只多了 $d$ 个已知分量。代价是维度增加；另一种等价做法是第 7 节的"模型前推 $\tau$"，计算更轻，延迟较大或不稳定时增广更严谨。

### 4.9 不可行与软约束（实车必须考虑）

硬约束 QP 可能**无解**：例如车被推出走廊、状态约束 $|e_y|\le0.2$ 而当前 $e_y=0.35$，任何输入序列都无法在第一步满足。实车上遇到无解不能直接输出空值。常见处理有两种：

1. **松弛变量**：把状态约束改成 $|e_{y,k}|\le0.2+s_k$，$s_k\ge0$，并在代价里加大惩罚 $\rho\,s_k^2$（例如 $\rho$ 取其它权重的 $10^2$–$10^4$ 倍）。QP 始终可行，约束被尽量满足，被迫违反时违反量最小。**输入约束（转向限幅）仍保持硬约束**，因为它代表物理极限。
2. **回退**：求解失败或超时时，沿用上一周期解的剩余部分（第 2 步起的输入平移一格），同时减速；连续失败超过阈值则停车。

**手算（教学数值）**：$e_y=0.35$，约束上界 0.2，松弛后需 $s\ge0.15$；若 $\rho=1000$，惩罚项 $=1000\times0.15^2=22.5$，远大于 4.3 节例子的跟踪代价 0.07，所以优化会优先把 $e_y$ 拉回走廊再考虑平顺。若 $\rho$ 只取 1，惩罚 0.0225 与跟踪代价同量级，约束基本被忽略——说明惩罚权重必须远大于其它项，软约束才起到"近似硬"的作用。

## 5. 横向 / 纵向解耦与速度规划

- **横向**：转向角控制，跟踪路径形状（上面三种）。
- **纵向**：速度控制（PID 或 MPC），跟踪速度曲线。
- **解耦的前提**：速度变化慢于横向动态；横向模型把 $v$ 当参数。耦合点：速度越大，同样转向角下横向加速度越大。
- **曲率限速**：$v\le\sqrt{a_{lat,max}/|\kappa|}$。例：$a_{lat,max}=1\ \text{m/s}^2$，$\kappa=1.0$ → $v\le1.0$ m/s；$\kappa=0.25$ → $2.0$ m/s。再叠加加减速限制和障碍距离限速，得到速度曲线。
- 联合 MPC 可同时优化 $v,\delta$，但计算更重。

### 5.1 解耦的速度曲线：曲率限速 + 前后向加速度限制（手算）

**问题**：只用曲率限速，速度在弯道入口处会"台阶式"下降，要求无限大的减速度。正确做法是在曲率限速上再做两遍扫描，使相邻点满足 $v_{i+1}^2\le v_i^2+2a\,\Delta s$。

**数据（教学数值）**：路径点间距 $\Delta s=0.5$ m，共 5 个点，曲率 $\kappa=[0,0,1.0,0,0]$ 1/m；直道速度上限 2 m/s；$a_{lat,max}=1$ m/s²，加速度和减速度限幅 $a=0.5$ m/s²。

1. 曲率限速：$v_i=\min(2,\sqrt{1/|\kappa_i|})$ → $[2,\ 2,\ 1,\ 2,\ 2]$。
2. 后向扫描（保证能减下来）：$v_i\le\sqrt{v_{i+1}^2+2a\Delta s}=\sqrt{v_{i+1}^2+0.5}$。从最后一个点往回：$v_4=2$；$i=3$：$\min(2,\sqrt{4+0.5})=2$；$i=2$：保持 1；$i=1$：$\min(2,\sqrt{1+0.5})=1.2247$；$i=0$：$\min(2,\sqrt{1.5+0.5})=1.4142$。结果 $[1.414,\ 1.225,\ 1,\ 2,\ 2]$。
3. 前向扫描（保证能加得上去）：$v_i\le\sqrt{v_{i-1}^2+0.5}$：$v_3=\min(2,\sqrt{1+0.5})=1.2247$；$v_4=\min(2,\sqrt{1.5+0.5})=1.4142$。
4. 最终速度曲线 $[1.414,\ 1.225,\ 1.0,\ 1.225,\ 1.414]$ m/s：弯心最慢，进出弯平滑，横向加速度 $v^2\kappa\le a_{lat,max}$ 在弯心恰好等于上限。

横向控制器（MPC / Pure Pursuit）把这条速度当作**已知参数**，这就是"解耦"成立的方式；若实车跟不上这条速度（加速度受限、打滑），要把实测速度反馈给横向模型，否则预测会错。

---

## 6. 参考轨迹生成

1. **最近点搜索**：找路径上距车最近的点（限制搜索窗口、保证单调前进，避免回环路径时跳到另一段）。
2. **弧长参数化**：用累计弧长 $s_i$ 作参数，便于按距离取前视点或 MPC 预测点。
3. **插值**：位置线性插值，航向取切线方向 $\theta_r=\operatorname{atan2}(\Delta y,\Delta x)$。
4. **曲率**：离散曲率 $\kappa_i\approx\Delta\phi/\Delta s$，需先平滑（原始点噪声会放大成曲率尖峰）。
5. **误差计算**：横向误差 $e_y=-(x-x_r)\sin\theta_r+(y-y_r)\cos\theta_r$（车在左侧为正）。
手算：最近点 $(2,\,1)$，$\theta_r=0$，车在 $(2.1,\,1.3)$：$e_y=-(0.1)\cdot0+(0.3)\cdot1=0.3$ m（左侧）。
6. **时间对齐**：见下面 6.1。
7. **倒车段**：航向参考与行驶方向反向，前视点取"身后"，符号需单独处理（与里程计符号问题同类，见[里程计章](../06_状态估计/04_里程计与传感器融合.md)）。

### 6.1 参考轨迹的时间对齐（MPC 特有的问题）

**问题**：局部规划（TEB）输出的是带时间戳的轨迹 $\{(t_i,x_i,y_i,\theta_i,v_i,\kappa_i)\}$，更新频率低于控制频率，且轨迹的 $t_i$ 是规划那一刻的相对时间。MPC 第 $k$ 步需要的是"此命令生效后第 $k$ 步"的参考点。如果直接按序号取第 $k$ 个点，就把"规划时刻"和"控制时刻"混为一谈，等于参考点比车快或慢。

**取点公式**：控制在时刻 $t_c$ 开始计算，指令延迟 $\tau$，MPC 步长 $\Delta t$，则第 $k$ 个预测点的参考时间为

$$
t_k=t_c+\tau+k\Delta t-t_{plan}
$$

其中 $t_{plan}$ 是这条轨迹的时间原点。在 $t_i\le t_k<t_{i+1}$ 之间线性插值：$w=(t_k-t_i)/(t_{i+1}-t_i)$。

**手算（教学数值）**：轨迹点 $t=0,\,0.5,\,1.0$ s，位置 $(0,0)$、$(0.5,0)$、$(0.98,0.12)$，航向 $0°,0°,14°$。若 $t_c-t_{plan}=0.3$ s、$\tau=0.2$ s，则第 0 个预测点 $t_0=0.5$ s，正好是轨迹点 2；第 3 个预测点（$\Delta t=0.1$）$t_3=0.8$ s：$w=(0.8-0.5)/0.5=0.6$，$x=0.5+0.6\times0.48=0.788$，$y=0.6\times0.12=0.072$，$\theta=0.6\times14°=8.4°$。

**如果不加 $\tau$ 直接按 $t_c$ 取点**：第 0 个点变成 $t=0.3$ s，位置 $(0.3,0)$。参考速度 1 m/s 时，沿路径方向差 $v\tau=0.2$ m——车永远在追一个落后 0.2 m 的目标，纵向表现为拖尾；在弯道上还会把纵向误差通过航向耦合成横向误差。

**航向插值的绕圈陷阱**：航向在 $\pm\pi$ 附近时，直接线性插值会出错。例：两个相邻点航向 $3.10$ 与 $-3.10$ rad（实际只差 0.083 rad），简单平均得 $0$，而正确值应接近 $\pi$。正确做法：先把差值归一化 $\Delta=\mathrm{wrap}(\theta_2-\theta_1)=0.083$，再 $\theta=\theta_1+w\Delta$（$w=0.5$ 得 $3.1416$）。

**状态更新时延**：MPC 的初始状态也要有对应的时间戳。若定位输出（EKF）的时间戳是 $t_s$，而当前计算时刻 $t_c>t_s$，先用 $t_c-t_s$ 对状态做前推，再叠加 $\tau$（第 7 节）。三个时间量不要混淆：**状态时间、计算时间、生效时间**。

---

## 7. 延迟及补偿

执行链有延迟 $\tau$（计算 + 通信 + 转向机构）。$v=1$ m/s、$\tau=0.2$ s，则车在指令生效前已走 $0.2$ m，转向指令按"旧状态"算会过量——表现为振荡。
**补偿**：用模型把当前状态向前预测 $\tau$：$x_p=x+v\cos\theta\,\tau$，$y_p=y+v\sin\theta\,\tau$，$\theta_p=\theta+\frac vL\tan\delta\,\tau$；以 $x_p$ 作为控制起点。MPC 中可把 $\tau/\Delta t$ 步的历史输入放入状态（增广，见 4.8 节），或预测起点后移。

**手算（教学数值）**：$v=1$ m/s，$L=0.5$ m，$\tau=0.2$ s；当前 $\theta=0.1$ rad，上一周期已发的转向角 $\delta=0.2$ rad（认为延迟期内保持不变）。

- 航向变化 $\Delta\theta=\dfrac{v}{L}\tan\delta\,\tau=2\times0.2027\times0.2=0.0811$ rad（4.6°）；
- 位置（欧拉，用旧航向）：$\Delta x=v\cos\theta\,\tau=0.199$ m，$\Delta y=v\sin\theta\,\tau=0.0200$ m；
- 位置（中点法，用半程航向 $\theta+\Delta\theta/2=0.1405$）：$\Delta x=0.198$ m，$\Delta y=0.0280$ m。

两者纵向几乎一样，横向差 8 mm；延迟越长、转向越大，中点法越值得用。**不补偿的后果**：控制器以为航向还是 0.1 rad，而实际它在指令生效时已是 0.181 rad，多出的 0.081 rad 会让横向误差变化率多 $v\times0.081=0.081$ m/s，这就是延迟引起的"过量转向、持续振荡"。

**延迟怎么得到**：把"计算耗时 + 通信 + 执行机构响应"分开测：计算耗时用时间戳，舵机响应可用阶跃命令到编码器反馈的时间差估计；然后取偏保守的 $\tau$ 值，设计时 $\tau$ 取整为若干个采样周期。

---

## 8. 在本项目里怎么用

CV：*MPC trajectory tracking: based on a car-like kinematic model, solved receding-horizon control under position, heading, speed and steering constraints, closing the loop from planned trajectory to vehicle motion on the real vehicle.* 对应：自行车模型（第 1、4 节）、位置/航向误差为状态（第 4.2 节）、速度和转向约束（第 4.1 节）、参考来自 TEB 的位姿与曲率（[TEB 章](./03_代价地图_局部规划_TEB.md) 第 9 节）。Pure Pursuit / Stanley 是对比基线与调试备份。见 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

具体配置（预测时域 $N$、步长、权重、求解器、控制频率、约束数值、延迟处理方式）的设计取值见 4.7 节；这些是量级与做法，调参时按实机微调。Pure Pursuit 与 MPC 的对比，落在机理差异上（切内角、转向平顺、约束行为），见 4.7 节最后一行。

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

### 9.0 终点与到达判据（容易被忽略的边界）

路径末端的前视点超出路径时，Pure Pursuit 会把目标钉在终点，$\alpha$ 随车靠近而剧烈变化；Stanley 的最近点在终点处不再前进。常见处理：距终点小于一个阈值（量级为车长的 0.5–1 倍）时，速度按 $v=\min(v_{ref},\sqrt{2a_{dec}d})$ 随剩余距离 $d$ 下降（例：$a_{dec}=0.5$，$d=0.5$ m → $v\le0.71$ m/s），到达容差（位置、航向各给一个阈值）满足后输出零速并保持，而不是继续跟踪噪声。MPC 同理，需要把参考速度序列在终点处衰减为 0，否则预测时域超过终点后参考点重复，会产生虚假的"追尾"代价。

### 9.1 现象到原因的排查顺序

1. **先看符号与坐标**：横向误差符号、航向差归一化、车头方向与行驶方向（倒车）；这类错误表现为"一开就发散"，调增益无效。
2. **再看时间**：状态时间戳、参考时间对齐、延迟补偿；表现为"低速正常、高速振荡"，且振荡周期约为 $4\times$ 延迟量级。
3. **再看约束**：转向是否长期饱和（日志中 $|\delta|$ 贴限幅）；饱和意味着路径曲率超出车辆能力，或前视 / 权重设置要求太激进。
4. **最后调参**：Pure Pursuit 调 $L_d$；Stanley 调 $k,k_s$；MPC 先调横向权重与转向速率权重，再动 $N$。

**为什么 MPC 只执行第一步**：设模型转向效率有 20% 误差，即真实 $\dot e_\psi$ 只有预测的 0.8 倍。若开环执行 4.3 节的 $(u_0,u_1)$，第 2 步的状态会偏离预测；滚动时域每周期用测量值重新作初始状态，误差不会累积到后续步。这就是"预测是为了决策，反馈是为了纠偏"。

**三种方法的选型一句话**：路径平滑、低速、无约束要求——Pure Pursuit；路径较直、要求横向精度和低速稳定——Stanley（配曲率前馈）；有转角 / 转角速率 / 速度约束、路径曲率变化大、需要显式利用参考曲率与时间——MPC。三者可以并存：MPC 失败（求解不可行、超时）时，退回几何法并减速，是常见的安全设计。

## 10. 面试问答

**Q1. Derive pure pursuit.**
> A: I pick a lookahead point at distance L_d on the path and require a circular arc through the rear axle that is tangent to the heading and passes through that point. Geometry gives curvature 2 sin alpha over L_d, and the steering angle is arctan of wheelbase times curvature. A larger L_d is smoother but cuts corners.

**Q2. Why MPC rather than Pure Pursuit?**
> A: MPC uses a model to look ahead and handles constraints explicitly, for example steering angle and rate limits and speed limits, and it can use reference curvature. Pure pursuit is simple and robust but purely geometric, so it cannot enforce constraints. In P5 I used MPC for the real vehicle, with a horizon of about one to two seconds, a step of about a tenth of a second, and a QP solver that finishes in milliseconds. The weights on cross-track and heading error are larger than the weights on steering effort and steering rate, and I compensate the actuation delay by predicting the state forward.

**Q3. What does the Stanley controller do?**
> A: It adds a heading-error term and a cross-track term, arctan of k times error over speed, at the front axle. The cross-track error decays exponentially, and the arctan keeps the command bounded.

**Q4. How do you handle control latency?**
> A: I predict the state forward by the latency using the kinematic model and compute the command from the predicted state, or I augment the MPC state with the commands still in the pipeline.

**Q5. Why is pure pursuit well damped?**

> A: For small errors on a straight path, the lateral error obeys a second-order equation with natural frequency root two times speed over lookahead and damping ratio one over root two, about 0.707. The damping ratio does not depend on speed or lookahead, but a longer lookahead makes the response slower and cuts corners.

**Q6. How do you build the QP in MPC?**

> A: I stack the predictions as X equals Phi x0 plus Gamma U, substitute into the cost, and get a quadratic in U with Hessian H equal to two times Gamma transpose Q Gamma plus R, and a linear term from x0. Input limits and rate limits become linear inequalities on U. Since H is positive definite the QP is convex and has a unique solution, and I apply only the first input.

**Q7. How do you align the reference with the controller time?**

> A: The reference trajectory has timestamps from the planner. For the prediction step k I query it at current time plus the actuation delay plus k times the step, with linear interpolation and wrapped angle differences. Otherwise the vehicle chases a point that is behind by speed times delay.

## 11. 速记、自测、链接

**速记**：$\kappa=2\sin\alpha/L_d$；$\delta=\arctan(L\kappa)$；Stanley $\delta=e_\psi+\arctan(ke/(v+k_s))$；横向误差指数收敛 $e^{-kt}$；MPC 只执行第一步；约束进优化；曲率限速 $\sqrt{a_{lat}/\kappa}$；延迟用模型前推。

**自测**：
1. $L=0.4,L_d=0.8,\alpha=20°$，求 $\delta$。（$\kappa=2\sin20°/0.8=0.855$；$\delta=\arctan0.342=18.9°$。）
2. Stanley $k=2,v=2,e=0.5,e_\psi=0$，$\delta$？（$\arctan0.5=26.6°$。）
3. $a_{lat,max}=2$，$\kappa=0.5$，最高速度？（$\sqrt4=2$ m/s。）
4. 为何 MPC 只执行第一个控制量？（模型和环境有误差，反馈重算比开环执行更稳。）
5. 车速 2 m/s、延迟 0.15 s，车在延迟内走多远？（0.3 m。）
6. Pure Pursuit 在 $v=2$ m/s、$L_d=2$ m 时的 $\omega_n$ 与 $\zeta$？（$\omega_n=\sqrt2\cdot2/2=1.414$ rad/s，$\zeta=0.707$。）
7. Stanley，$k=1.5$，$v=1$，$k_s=0.5$，$e=0.3$ m，$e_\psi=0$，$\delta$？（$\arctan0.3=16.7°$。）
8. 4.3 节的 QP 里 $H$ 与 $f$ 是什么？（$H=\begin{bmatrix}2.7&1\\1&1.2\end{bmatrix}$，$f=[0.2,0]^\top$。）
9. 速率约束例子里，为什么两条约束都激活？（无约束解 $u_0=-0.107$ 低于下界 $-0.06$，且 $u_1-u_0$ 超过 0.08；KKT 乘子 0.022、0.036 都非负。）
10. $\tau=0.2$ s、$\Delta t=0.1$ s，增广状态增加几维？（2 维，$u_{k-2},u_{k-1}$。）
11. $L=0.5$，$\kappa_r=1$，线性化后 $B$ 的航向分量比直线时大多少倍？（$1+L^2\kappa^2=1.25$ 倍。）

**链接**：上一章 [代价地图、局部规划与 TEB](./03_代价地图_局部规划_TEB.md)；下一章 [导航系统架构](./05_导航系统架构_Nav2与move_base对比.md)；[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)；[路径规划与导航八股](../../03_八股知识/03_机器人学/04_路径规划与导航八股.md)。
