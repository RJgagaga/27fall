# 10 MPC 轨迹跟踪

> **一句话定位**：MPC（Model Predictive Control，模型预测控制）每个控制周期做同一件事：用车的运动学模型向前预测未来 $N$ 步，在位置/航向/速度/转角约束下，找一串使"偏离参考轨迹 + 用力过猛"最小的控制量；只执行第一步，下一周期重来（滚动时域，receding horizon）。CV 对应：*based on a car-like kinematic model, solved receding-horizon control under position, heading, speed and steering constraints, closing the loop from planned trajectory to vehicle motion on the real vehicle*。

**本章学完你能：**

- 讲清滚动时域的思想，以及"$N$ 个动作、$N+1$ 个状态"为什么；
- 写出车式运动学模型 $(x,y,\psi,v)$、输入 $(a,\delta)$，并推导雅可比 $A,B$、离散化、误差模型；
- **手算一个预测步**，并手算一个两步的优化问题，明确"**优化器自己选择动作**，你不需要给具体动作"（笔记里反复问到的点）；
- 解释代价函数各项（状态误差、输入、输入变化、终端代价 $Q_f$）和权重含义，以及怎样选权重；
- 写出约束（输入界、变化率、速度、转角、安全走廊、软约束与松弛变量）；
- 把问题整理成 QP，解释求解器如何工作（有效集/内点/ADMM），热启动；
- 处理**参考轨迹的时间对齐**（TEB 轨迹带时间）和**延迟补偿**，并手算；
- 排查 MPC 抖动、拐角切内、跟踪滞后、求解失败；
- 与 Pure Pursuit、Stanley、LQR 比较。

**前置知识：**

- MPC 基础：[../../02_基础知识/05_控制理论/05_MPC模型预测控制.md](../../02_基础知识/05_控制理论/05_MPC模型预测控制.md)
- 最优化与二次规划：[../../02_基础知识/01_数学基础/04_最优化与二次规划.md](../../02_基础知识/01_数学基础/04_最优化与二次规划.md)
- 数值方法与离散化：[../../02_基础知识/01_数学基础/05_数值方法与离散化.md](../../02_基础知识/01_数学基础/05_数值方法与离散化.md)
- 轨迹跟踪（几何控制器对比）：[../../02_基础知识/08_规划与导航/04_轨迹跟踪_PurePursuit_Stanley_MPC.md](../../02_基础知识/08_规划与导航/04_轨迹跟踪_PurePursuit_Stanley_MPC.md)
- 本项目：[02 运动学](./02_车辆运动学与里程计.md)、[09 TEB](./09_TEB局部轨迹优化.md)

**目录**

1. 要解决什么问题
2. 滚动时域：$N$ 个动作、$N+1$ 个状态
3. 车式运动学模型
4. 线性化与离散化：手算一个预测步
5. 代价函数与权重
6. 约束
7. 整理成 QP：手算两步优化
8. 求解器如何选动作
9. 参考轨迹的时间对齐
10. 延迟补偿
11. 实机闭环结构与安全
12. 调参方法
13. 与其他跟踪器的比较
14. 失效模式与故障现象
15. 在 ROS2 里实现 MPC 节点：结构、话题、定时器与时间预算
16. 面试问答 / 要点速记 / 自测题 / 链接

---

## 1. 要解决什么问题

TEB（[09 章](./09_TEB局部轨迹优化.md)）给出一条漂亮的带时间轨迹。但真实车：

- 转向舵机有**速率限制和滞后**；
- 速度环有**响应时间**；
- 从发出命令到车轮响应有**延迟**；
- 模型有误差、状态有噪声。

直接"取第一段 $v,\omega$ 发给底盘"不会给出好的跟踪。需要一个控制器：**知道车怎么动（模型）、知道有哪些限制（约束）、知道要去哪里（参考）、并且会往前看一小段**。MPC 就是这样的控制器。它特别适合车式机器人，因为它能把"不能超过最大转角/转角速率、速度上下限（含倒车）、不能偏离走廊太远"直接写进问题里，而不是靠事后限幅。

---

## 2. 滚动时域：$N$ 个动作、$N+1$ 个状态

每个控制周期（采样时间 $\Delta t$，例如 0.05–0.1 s）：

1. 测得当前状态 $\mathbf x_0$；
2. 向前预测 $N$ 步，决策变量是 $N$ 个控制量 $\mathbf u_0,\mathbf u_1,\ldots,\mathbf u_{N-1}$；
3. 由模型 $\mathbf x_{k+1}=f(\mathbf x_k,\mathbf u_k)$ 推出 $N+1$ 个状态 $\mathbf x_0,\mathbf x_1,\ldots,\mathbf x_N$；
4. 求使代价最小的 $\mathbf u_{0:N-1}$；
5. **只把 $\mathbf u_0$ 发给车**；
6. 下一周期，用新的测量重复。

**为什么是 $N$ 个动作、$N+1$ 个状态？**

```text
时间:  t0      t1      t2     ...    t(N-1)    tN
状态:  x0      x1      x2     ...    x(N-1)    xN        (N+1 个)
动作:     u0      u1      u2   ...        u(N-1)          (N 个)
          └──走一步──┘
```

每个动作"走一步"，把一个状态变成下一个状态。$N$ 步需要 $N$ 个动作，端点算上共 $N+1$ 个状态。$\mathbf x_0$ 是**测量值**（已知，不是决策变量），所以真正被预测的是 $\mathbf x_1\ldots\mathbf x_N$。

预测时域 $T_p=N\Delta t$（教学示例 $N=10$、$\Delta t=0.1$ → 1 s）。时域太短：看不到弯，转弯晚；太长：计算量大、对模型误差敏感。

---

## 3. 车式运动学模型

状态 $\mathbf x=[x,\ y,\ \psi,\ v]^\top$，输入 $\mathbf u=[a,\ \delta]^\top$（纵向加速度，前轮等效转角）。以后轴中心为参考点（见 [02 章](./02_车辆运动学与里程计.md)）：

$$\dot x=v\cos\psi,\quad \dot y=v\sin\psi,\quad \dot\psi=\frac{v}{L}\tan\delta,\quad \dot v=a.$$

> **说明**：也可以把 $(v,\delta)$ 当输入、状态只有 $(x,y,\psi)$；或把 $\delta$ 当状态、转角速率 $\dot\delta$ 当输入（可直接约束转角速率并建模转向滞后）。输入形式选哪个取决于底层接口接受什么指令。本项目采用 $\mathbf x=[x,y,\psi,v]^\top$（$n_x=4$）、$\mathbf u=[a,\delta]^\top$（$n_u=2$），下位机接收的是速度与转角指令，所以对 MPC 输出的 $a$ 积分一步得到 $v$ 指令。

**倒车**：$v<0$，模型自动给出相反的航向变化；约束里速度下界为负值即可（但线性化点的符号要对）。

---

## 4. 线性化与离散化：手算一个预测步

### 4.1 为什么线性化

模型含 $\cos\psi,\sin\psi,\tan\delta$，非线性；非线性优化（NMPC）可做但慢，且实时性难保。常用做法：围绕**参考轨迹**线性化，得到随时间变化的线性模型（LTV），问题变成**凸二次规划（QP）**，快而稳。

### 4.2 雅可比

参考点 $(x_r,y_r,\psi_r,v_r,\delta_r)$（参考轨迹上的状态与输入，$v_r,\delta_r$ 可由 TEB 的 $v_i$、$\kappa_i$ 得到：$\delta_r=\arctan(L\kappa_r)$）。

$$A=\frac{\partial f}{\partial\mathbf x}=\begin{bmatrix}0&0&-v_r\sin\psi_r&\cos\psi_r\\0&0&v_r\cos\psi_r&\sin\psi_r\\0&0&0&\frac{\tan\delta_r}{L}\\0&0&0&0\end{bmatrix},\qquad B=\frac{\partial f}{\partial\mathbf u}=\begin{bmatrix}0&0\\0&0\\0&\frac{v_r}{L\cos^2\delta_r}\\1&0\end{bmatrix}.$$

（$B$ 的两列依次对应 $a$、$\delta$。）逐元素推导：$\partial(v\cos\psi)/\partial\psi=-v\sin\psi$，$\partial(v\cos\psi)/\partial v=\cos\psi$，$\partial(v\tan\delta/L)/\partial v=\tan\delta/L$，$\partial(v\tan\delta/L)/\partial\delta=v/(L\cos^2\delta)$（因 $(\tan\delta)'=1/\cos^2\delta$），$\partial\dot v/\partial a=1$。

### 4.3 误差模型

定义偏差 $\tilde{\mathbf x}=\mathbf x-\mathbf x_r$，$\tilde{\mathbf u}=\mathbf u-\mathbf u_r$。参考轨迹满足模型，所以偏差近似满足
$$\dot{\tilde{\mathbf x}}\approx A\tilde{\mathbf x}+B\tilde{\mathbf u}.$$
这样 MPC 要把**偏差调到 0**（调节问题），代价函数自然就是偏差的二次型。

### 4.4 离散化

欧拉：$A_d=I+A\Delta t$，$B_d=B\Delta t$。更准的是零阶保持（ZOH）：$A_d=e^{A\Delta t}$，$B_d=\int_0^{\Delta t}e^{A\tau}d\tau\,B$。小步长下欧拉足够；步长大或速度高则用 ZOH/RK4（见基础文档）。

### 4.5 完整手算：一个预测步

**设定（教学数值）**：$L=0.5$，$\Delta t=0.1$；参考点：$\psi_r=0$，$v_r=1.0$，$\delta_r=0$（直行）。

$$A=\begin{bmatrix}0&0&0&1\\0&0&1&0\\0&0&0&0\\0&0&0&0\end{bmatrix},\ \ B=\begin{bmatrix}0&0\\0&0\\0&2\\1&0\end{bmatrix}\ \ (\tfrac{v_r}{L\cos^2\delta_r}=\tfrac{1}{0.5}=2).$$
$$A_d=I+A\Delta t=\begin{bmatrix}1&0&0&0.1\\0&1&0.1&0\\0&0&1&0\\0&0&0&1\end{bmatrix},\qquad B_d=\begin{bmatrix}0&0\\0&0\\0&0.2\\0.1&0\end{bmatrix}.$$
（此处 $A^2=0$，ZOH 与欧拉相同。）

当前偏差（测量减参考）$\tilde{\mathbf x}_0=[\tilde x,\tilde y,\tilde\psi,\tilde v]=[0,\ 0.10,\ 0.05,\ -0.10]$：车在参考的左侧 0.10 m，航向偏左 0.05 rad，速度比参考慢 0.1 m/s。施加偏差输入 $\tilde{\mathbf u}_0=[\tilde a,\tilde\delta]=[0.2,\ -0.1]$（加速一点，前轮向右打一点）：

$$\tilde{\mathbf x}_1=A_d\tilde{\mathbf x}_0+B_d\tilde{\mathbf u}_0:$$
- $\tilde x_1=0+0.1\times(-0.10)=-0.010$；
- $\tilde y_1=0.10+0.1\times0.05=0.105$（航向偏左让横向偏差继续变大）；
- $\tilde\psi_1=0.05+0.2\times(-0.1)=0.030$（向右打舵让航向回正）；
- $\tilde v_1=-0.10+0.1\times0.2=-0.080$。

$\tilde{\mathbf x}_1=[-0.010,\ 0.105,\ 0.030,\ -0.080]$。

**一步的代价**（示例权重 $Q=\mathrm{diag}(1,10,5,1)$，$R=\mathrm{diag}(1,2)$）：
状态：$1\times0.0001+10\times0.011025+5\times0.0009+1\times0.0064=0.0001+0.11025+0.0045+0.0064=0.1213$；输入：$1\times0.2^2+2\times0.1^2=0.04+0.02=0.06$。

优化的任务，就是在 $N$ 步上同时选好 $\tilde{\mathbf u}_0\ldots\tilde{\mathbf u}_{N-1}$，让这些代价之和最小。

---

## 5. 代价函数与权重

$$J=\underbrace{\sum_{k=0}^{N-1}\Big(\tilde{\mathbf x}_k^\top Q\tilde{\mathbf x}_k+\tilde{\mathbf u}_k^\top R\tilde{\mathbf u}_k+\Delta\mathbf u_k^\top R_d\Delta\mathbf u_k\Big)}_{\text{阶段代价}}+\underbrace{\tilde{\mathbf x}_N^\top Q_f\tilde{\mathbf x}_N}_{\text{终端代价}}.$$

| 项 | 含义 | 权重大的效果 |
|---|---|---|
| $Q$ | 位置、航向、速度偏差 | 跟得紧，但转向更激进 |
| $R$ | 控制量大小（加速度、转角） | 动作小、平缓，但响应慢 |
| $R_d$ | 相邻控制量变化 $\Delta\mathbf u_k=\mathbf u_k-\mathbf u_{k-1}$ | 抑制抖动、保证平顺 |
| $Q_f$ | 终端状态偏差 | 保证预测窗末端回到参考，影响稳定性 |

### 5.1 权重怎么选（量纲）

各状态单位不同（m、rad、m/s），权重要把它们变成可比的量。一个入门规则（Bryson 法）：权重 $\approx 1/(\text{可接受最大值})^2$。

> **示例**：可接受横向误差 0.2 m → $Q_y=1/0.2^2=25$；航向 0.2 rad → $Q_\psi=25$；速度误差 0.3 m/s → $Q_v=11$；转角 0.5 rad → $R_\delta=4$；加速度 1.0 m/s² → $R_a=1$。然后根据现象微调：想更紧的横向跟踪，把 $Q_y$ 乘 2–4 倍；转向抖，增大 $R_d$。

### 5.2 终端代价 $Q_f$ 为什么存在，一般多大

有限时域的优化只"看" $N$ 步。如果终端不惩罚，优化器会让前面几步很舒服、把麻烦留给窗口以外，最终导致不稳定或"短视"。理论上，$Q_f$ 取无限时域 LQR 的代价矩阵 $P$（解 Riccati 方程），则有限时域代价等价无限时域，稳定性有保证。

> **示例手算（一维玩具）**：模型 $\tilde\psi_{k+1}=\tilde\psi_k+0.2\tilde\delta_k$（$A=1,B=0.2$），$q=5,\ r=2$。离散代数 Riccati：
> $$P=q+A^2P-\frac{(ABP)^2}{r+B^2P}=5+P-\frac{0.04P^2}{2+0.04P}\ \Rightarrow\ 0.04P^2-0.2P-10=0\ \Rightarrow\ P=\frac{0.2+\sqrt{0.04+1.6}}{0.08}=18.5.$$
> 所以终端权重取 $Q_f\approx18.5$，**比阶段权重 $q=5$ 大约 3.7 倍**。这回答了"$Q_f$ 一般多少"：没有通用常数，一种原则性的取法就是解 Riccati 的 $P$；工程上常取 $Q_f=(1\sim10)Q$，再用仿真/实机验证。

---

## 6. 约束

### 6.1 约束类型

| 类别 | 形式 | 例子 |
|---|---|---|
| 输入界 | $\mathbf u_{min}\le\mathbf u_k\le\mathbf u_{max}$ | $|\delta|\le\delta_{max}=30^\circ$（对应最小转弯半径）；$|a|\le a_{max}$ |
| 输入变化率 | $|\mathbf u_k-\mathbf u_{k-1}|\le\dot{\mathbf u}_{max}\Delta t$ | $|\Delta\delta|\le\dot\delta_{max}\Delta t$（转向机构速率限制） |
| 状态界 | $\mathbf x_{min}\le\mathbf x_k\le\mathbf x_{max}$ | $v_{min}\le v\le v_{max}$（含倒车下界）；曲率限速 |
| 位置走廊 | 线性化后的半平面 $\mathbf n^\top\mathbf p_k\le b$ | 不越出安全带 |
| 终端 | $\mathbf x_N\in\mathcal X_f$ | |

> **示例**：转角界 $|\delta|\le30^\circ=0.5236$ rad；转角速率 $60^\circ/\mathrm s$ → 每 0.1 s 最多变化 $6^\circ=0.1047$ rad。因为模型里用的是偏差变量，约束要换算：$\tilde\delta_k\in[\delta_{min}-\delta_{r,k},\ \delta_{max}-\delta_{r,k}]$。

### 6.2 硬约束与软约束

- **硬约束**：必须满足；若问题不可行（例如当前状态已越界、约束互相冲突），求解器返回"无解"，这在实车上很危险；
- **软约束**：加松弛变量 $\varepsilon\ge0$，放宽约束 $g(\mathbf x)\le\varepsilon$，并在代价中加 $\rho\varepsilon^2$（或 $\rho\varepsilon$）。对状态约束（速度、位置走廊）常用软约束，保证总有解。**输入界通常保持硬约束**（执行器的物理极限）。

---

## 7. 整理成 QP：手算两步优化

### 7.1 把预测"摊平"

把所有预测状态堆成向量 $\tilde X=[\tilde{\mathbf x}_1;\ldots;\tilde{\mathbf x}_N]$，输入堆成 $U=[\tilde{\mathbf u}_0;\ldots;\tilde{\mathbf u}_{N-1}]$。由线性模型递推：
$$\tilde X=\Phi\,\tilde{\mathbf x}_0+\Gamma\,U,$$
其中 $\Phi$ 的第 $k$ 块是 $A_d^k$（时变时为连乘），$\Gamma$ 是下三角分块矩阵，第 $(k,j)$ 块为 $A_d^{k-j}B_d$（$k\ge j$，时变时为相应连乘）。

代价（忽略 $R_d$、$Q_f$ 作简化）： $J=\tilde X^\top\bar Q\tilde X+U^\top\bar RU$，代入：
$$J=U^\top\underbrace{(\Gamma^\top\bar Q\Gamma+\bar R)}_{H'}U+2\underbrace{(\Gamma^\top\bar Q\Phi\tilde{\mathbf x}_0)}_{g}{}^\top U+\text{const}.$$
标准 QP 形式 $\min_U\ \tfrac12U^\top HU+f^\top U$ 中，$H=2H'$，$f=2g$。再加上约束 $G U\le h$。

**这个形式说明了一个关键事实：决策变量是 $U$，$\tilde{\mathbf x}_0$ 只是进入 $f$ 的"参数"。优化器输出的是 $U$。**

### 7.2 手算：一个标量、两步

**设定（教学数值）**：只看航向偏差，模型 $\tilde\psi_{k+1}=\tilde\psi_k+0.2\tilde\delta_k$；当前 $\tilde\psi_0=0.05$；$q=5$（惩罚 $\tilde\psi_1,\tilde\psi_2$），$r=2$（惩罚 $\tilde\delta_0,\tilde\delta_1$）。$N=2$。

预测：$\tilde\psi_1=\tilde\psi_0+0.2u_0$，$\tilde\psi_2=\tilde\psi_0+0.2u_0+0.2u_1$。所以
$$\Phi=\begin{bmatrix}1\\1\end{bmatrix},\quad\Gamma=\begin{bmatrix}0.2&0\\0.2&0.2\end{bmatrix}.$$
$$J=5(\tilde\psi_1^2+\tilde\psi_2^2)+2(u_0^2+u_1^2).$$

令 $\partial J/\partial U=0$：$(q\Gamma^\top\Gamma+rI)U=-q\Gamma^\top\Phi\,\tilde\psi_0$。

- $\Gamma^\top\Gamma=\begin{bmatrix}0.08&0.04\\0.04&0.04\end{bmatrix}$，$q\Gamma^\top\Gamma+rI=\begin{bmatrix}0.4+2&0.2\\0.2&0.2+2\end{bmatrix}=\begin{bmatrix}2.4&0.2\\0.2&2.2\end{bmatrix}$；
- $\Gamma^\top\Phi=[0.4,\ 0.2]^\top$，右端 $=-5\times0.05\times[0.4,0.2]^\top=[-0.1,\ -0.05]^\top$；
- $\det=2.4\times2.2-0.04=5.24$；$u_0=\dfrac{2.2(-0.1)-0.2(-0.05)}{5.24}=-0.0401$，$u_1=\dfrac{2.4(-0.05)-0.2(-0.1)}{5.24}=-0.0191$。

**最优动作序列** $U^*=[-0.0401,\ -0.0191]$。预测：$\tilde\psi_1=0.05-0.0080=0.0420$，$\tilde\psi_2=0.0420-0.0038=0.0382$。

**滚动**：车只执行 $u_0=-0.0401$ rad（向右打约 2.3°）。下一周期测得新的 $\tilde\psi_0'$，重新解一遍。

### 7.3 加上约束：有效集

若转角偏差限幅 $|\tilde\delta|\le0.03$：无约束解 $u_0=-0.0401$ 越界，$u_0$ 被"顶"在下界 $-0.03$。固定 $u_0=-0.03$，对 $u_1$ 求最优：第二个方程 $0.2u_0+2.2u_1=-0.05\Rightarrow u_1=(-0.05+0.006)/2.2=-0.0200$。

**检验（KKT）**：此时 $\partial J/\partial u_0=2.4u_0+0.2u_1+0.1=-0.072-0.004+0.1=+0.024>0$，意味着减小 $u_0$ 还能降代价，但被下界拦住，所以这个约束是"有效的"，解是对的。这就是**有效集方法**的思想：猜哪些约束起作用，把它们当等式，解剩下的，再检验乘子符号。

### 7.4 一段数值示意代码（教学示例，非项目源码）

```python
import numpy as np

def build_prediction(Ad, Bd):
    """Ad, Bd: lists (length N) of discrete LTV matrices. Returns Phi, Gamma."""
    N, nx, nu = len(Ad), Ad[0].shape[0], Bd[0].shape[1]
    Phi = np.zeros((N * nx, nx))
    Gam = np.zeros((N * nx, N * nu))
    A_pow = np.eye(nx)
    for k in range(N):
        A_pow = Ad[k] @ A_pow
        Phi[k*nx:(k+1)*nx, :] = A_pow
    for j in range(N):
        M = Bd[j]
        for k in range(j, N):
            if k > j:
                M = Ad[k] @ M
            Gam[k*nx:(k+1)*nx, j*nu:(j+1)*nu] = M
    return Phi, Gam

def solve_unconstrained(Phi, Gam, x0, Qbar, Rbar):
    H = Gam.T @ Qbar @ Gam + Rbar
    g = Gam.T @ Qbar @ Phi @ x0
    return -np.linalg.solve(H, g)          # optimal stacked inputs U

# scalar two-step example from the text
Phi = np.array([[1.0], [1.0]]); Gam = np.array([[0.2, 0.0], [0.2, 0.2]])
U = solve_unconstrained(Phi, Gam, np.array([0.05]), 5*np.eye(2), 2*np.eye(2))
print(U)   # -> [-0.0401, -0.0191]
```

---

## 8. 求解器如何选动作

这一节集中回答你笔记里反复追问的问题："MPC 的动作怎么选？选哪些数值？是不是我给一些候选动作让它挑？"

**答：不是。** 你**不需要**给任何具体动作，也不是采样挑选。流程是：

1. **你给优化器的是**：模型（$A_d,B_d$）、代价权重（$Q,R,R_d,Q_f$）、约束（输入界、变化率、状态界）、当前状态 $\tilde{\mathbf x}_0$、参考轨迹，以及上一周期实际执行的输入 $\mathbf u_{-1}$（用来算 $\Delta\mathbf u_0$ 和变化率约束）；
2. **优化器返回的是**：整串 $U^*$（$N$ 个 $(a,\delta)$）；
3. **选哪个数值**由"代价最小且满足约束"这个数学条件唯一（凸 QP 时）决定。

**"上一次的动作"的作用有两个**：（a）作为 $\Delta u$ 项和速率约束的起点；（b）**热启动**——把上一周期的解整体前移一步（丢掉 $\mathbf u_0$，末尾补一个）作为本周期的初始猜测，让迭代求解器更快收敛。它不是"候选动作"。

### 8.1 三类求解器

| 方法 | 原理 | 特点 | 常见实现 |
|---|---|---|---|
| 有效集（active set） | 猜哪些不等式取等号，解等式约束 QP，检查乘子并更新 | 小问题快、精度高、热启动好 | qpOASES |
| 内点法（interior point） | 用对数障碍函数把不等式并入目标，牛顿迭代 | 迭代数稳定，适合大一点问题 | HPIPM、CVXGEN、IPOPT（非线性） |
| ADMM / 一阶方法 | 变量分裂、交替更新 | 容易实现、热启动；精度中等 | OSQP |

车式小车典型规模：$N=10$–$20$，$n_x=4$，$n_u=2$，凝聚后决策变量 20–40 个，求解通常是毫秒量级（教学量级，本项目使用 OSQP 并热启动，$N=15$ 时单次求解约 2–5 ms）。

### 8.2 稀疏形式与凝聚形式

- **凝聚（condensed）**：消去状态，只保留 $U$，矩阵稠密但小（上面的 $H$）；
- **稀疏（sparse）**：把 $\mathbf x_k$ 也留作变量，用动力学作为等式约束，矩阵稀疏、块带状；$N$ 大时更优。

### 8.3 NMPC 与 LTV-MPC

不线性化、直接用非线性模型求解的是 NMPC（需要 SQP/内点法，如 CasADi + IPOPT、acados）。LTV-MPC 用参考轨迹逐步线性化，成 QP，更快更稳；代价是离参考较远时线性化误差变大。选择取决于算力、偏差大小、实时性要求。

---

## 9. 参考轨迹的时间对齐

MPC 每一步 $k$ 需要参考 $\mathbf x_{r,k}$ 与 $\mathbf u_{r,k}$，对应时刻 $t_{now}+k\Delta t$。TEB 的轨迹是**离散点，时间间隔 $\Delta T_i$ 不均匀**。必须**按时间重采样**。

### 9.1 两种索引方式

- **按时间**：参考点 = 轨迹在 $t_{now}+k\Delta t$ 时刻的插值。问题：如果 $t_{now}$ 的基准不一致（轨迹时间原点 ≠ 控制时间），整条参考会前后错位，车表现为"总是追不上"或"冲过头"。
- **按弧长/最近点**：先找车在轨迹上的投影点，再沿轨迹向前按 $v_r\Delta t$ 逐步取点。对时间基准不敏感，对速度更宽容；缺点是丢掉了"在什么时刻到"的信息（对要求准时的任务不适合）。

很多实现用"最近点 + 按轨迹速度推进"的混合方式。

### 9.2 插值要点与手算

- **角度**：对航向插值要先 unwrap，再线性插值，再归一化；
- **位置**：相邻点间距大时，用直线插值会产生**弦误差**。

> **示例手算**：用 [09 章](./09_TEB局部轨迹优化.md) 的轨迹：P0 $(0,0,0^\circ)$ 在 $t=0$，P1 $(1.5,0.45,33.4^\circ)$ 在 $t=1.57$，段内为圆弧（半径 $R=2.724$，总转角 $0.583$ rad）。取 $t=0.5$，比例 $f=0.5/1.57=0.3185$。
> - **直线插值**：$x=0.478,\ y=0.143,\ \psi=10.6^\circ$；
> - **圆弧插值**：转角 $\psi=0.3185\times0.583=0.1857$ rad $=10.6^\circ$；$x=R\sin\psi=2.724\times0.1846=0.503$，$y=R(1-\cos\psi)=2.724\times0.01719=0.047$。
> - $y$ 方向差 $0.143-0.047=0.096\ \mathrm m$——近 10 cm。原因是这个示例的点距（1.5 m）很大。实际 TEB 的时间分辨率更细（`dt_ref` 常为零点几秒，点距更小），弦误差小得多；但如果你把参考点取得稀，或把低频轨迹直接线性插值，就会出现"沿直线抄近路"的拐角切内。**解决：提高轨迹采样密度，或按曲率/航向做圆弧（或三次 Hermite）插值。**

### 9.3 速度/曲率前馈

参考输入 $\mathbf u_r$ 由轨迹给出：$\delta_{r,k}=\arctan(L\kappa_{r,k})$，$a_{r,k}=(v_{r,k+1}-v_{r,k})/\Delta t$。误差模型里 MPC 求的是偏差，最终指令 $\mathbf u=\mathbf u_r+\tilde{\mathbf u}$，**前馈 + 反馈**，这样直线与稳态弯道上偏差输入几乎为 0，抖动小。

### 9.4 反向、cusp

倒车段 $v_r<0$；到 cusp 时参考速度先降到 0 再反向。线性化中 $v_r$ 的符号要对，否则 $B$ 的符号反了（$B_{\psi,\delta}=v_r/(L\cos^2\delta_r)$）。

---

## 10. 延迟补偿

### 10.1 延迟从哪来

状态采集时间 + EKF 周期 + 求解时间 + 通信 + 底层周期 + 执行器响应。总和记作 $\tau$。MPC 在 $t$ 时刻基于 $\mathbf x(t)$ 算出的命令，实际作用在 $t+\tau$ 时刻的车上，那时车已经不在 $\mathbf x(t)$ 了。

### 10.2 影响（手算）

> **示例**：$\tau=0.15$ s，$v=1.0$ m/s，$\omega=0.5$ rad/s。
> - 位置差：$v\tau=0.15$ m；
> - 航向差：$\omega\tau=0.075$ rad $=4.3^\circ$；若前视 1 m，则等价于 7.5 cm 的横向误差；
> - 相位滞后：一个 1 Hz 的振荡（$\omega_o=6.28$ rad/s）在 0.15 s 延迟下相位滞后 $6.28\times0.15=0.94$ rad $=54^\circ$，显著降低相位裕度，闭环容易发散成持续振荡。

### 10.3 补偿方法

1. **状态预测（最常用）**：把测量状态用模型向前推 $\tau$，用推出的状态作为 MPC 的 $\mathbf x_0$：$\hat{\mathbf x}(t+\tau)=f_\tau(\mathbf x(t),\ \text{这 }\tau\text{ 内已发出但未生效的命令})$。需要缓存最近若干周期的已发命令；
2. **状态增广**：把延迟的命令作为附加状态加进模型（每多一个采样延迟多一组状态）；
3. **减小延迟**：提高 EKF/控制频率、减少通信缓冲、测量并校准延迟。

> **示例**：采样 0.05 s，$\tau=0.15$ → 3 个采样延迟。缓存最近 3 个命令 $[\mathbf u_{-3},\mathbf u_{-2},\mathbf u_{-1}]$，从测量状态依次用 3 步模型推出 $\hat{\mathbf x}$，再解 MPC，并且 MPC 的参考也要对应从 $t+\tau$ 开始取点（时间对齐，第 9 节）。

### 10.4 转向机构滞后

转向舵机近似一阶惯性 $\dot\delta=(\delta_{cmd}-\delta)/T_s$。若 $T_s=0.1$ s，发出阶跃命令后 0.1 s 只到达 63%，0.3 s 到达 95%。MPC 如果把 $\delta$ 当作瞬时可得，会过度修正。处理：把 $\delta$ 作为状态、$\dot\delta$ 或 $\delta_{cmd}$ 作为输入，并把一阶滞后写进模型；或者给转角速率加约束。

---

## 11. 实机闭环结构与安全

```text
                    ┌──────────────────────────────────────────┐
 TEB 轨迹 ─────────►│ 参考生成: 时间对齐 / 插值 / 曲率前馈 u_r   │
 (t,x,y,ψ,v,κ)      └───────────────┬──────────────────────────┘
                                    ▼
 状态估计(EKF) ──► 延迟补偿 ──► MPC: 线性化 → QP → 求解 → u* ──► 限幅/斜坡 ──► 底层
     ▲                              (热启动)                              │
     └──────────────────── 车辆运动 (传感器反馈) ◄────────────────────────┘
```

**必备的安全措施**（典型做法）：

1. **求解失败回退**：QP 无解/超时 → 沿用上一周期解的下一步（移位），连续失败 → 减速停车；
2. **输入再限幅与斜坡**：最后一道硬保护（不要只依赖 MPC 内部约束）；
3. **状态有效性**：状态过期、协方差过大、跳变 → 降速或停；
4. **参考异常**：轨迹过期、不连续 → 停；
5. **看门狗/超时**：下位机若 $T$ 内未收到命令则刹车；
6. **曲率限速**：$v\le\sqrt{a_{y,max}/|\kappa|}$。

---

## 12. 调参方法

1. **模型对**：先开环验证模型：给固定 $(v,\delta)$ 让车跑圆，量半径，核对 $\delta$ 映射与 $L$（[02 章](./02_车辆运动学与里程计.md) 10.2 节）；
2. **从低速开始**：低速、直线，只开横向误差权重，观察响应；
3. **权重量纲**：用 Bryson 规则设初值（第 5.1 节），再调；
4. **时域**：$T_p$ 覆盖转弯所需距离（至少看到下一个弯）；$N$ 太大算不过来，减 $N$ 而非减 $\Delta t$ 太多；
5. **平滑**：加大 $R_d$ 或转角速率约束；
6. **延迟**：测 $\tau$ 并补偿；
7. **逐项验证**：看实际转角响应曲线、横向误差曲线、求解时间分布，而不是只看"跑得好不好看"。

| 现象 | 调整方向 |
|---|---|
| 横向误差大，但转向平稳 | 增大 $Q_y$、$Q_\psi$ |
| 转向抖 | 增大 $R_d$、$R_\delta$；降低状态噪声；检查延迟 |
| 弯道外扩 | 提高曲率前馈质量；加长时域；检查参考时间对齐 |
| 弯道切内 | 参考点太稀（弦误差）；$Q$ 对位置权重过大而对航向小 |
| 速度跟不上 | 增大 $Q_v$；检查加速度限幅 |
| 一启动就猛打方向 | 初始偏差大 + $Q$ 大；用转角速率约束；先消除初始偏差 |

---

## 13. 与其他跟踪器的比较

| 项 | Pure Pursuit | Stanley | LQR | MPC |
|---|---|---|---|---|
| 核心 | 前视点 + 几何圆弧 | 航向误差 + 横向误差项 | 线性模型、无限时域 | 有限时域滚动优化 |
| 公式 | $\delta=\arctan(2L\sin\alpha/l_d)$ | $\delta=\psi_e+\arctan(ke/v)$ | $\mathbf u=-K\tilde{\mathbf x}$ | 解 QP |
| 约束 | 无（事后限幅） | 无 | 无（事后限幅） | 显式 |
| 预测 | 无 | 无 | 隐含（无限时域） | 显式（$N$ 步） |
| 延迟补偿 | 难 | 难 | 可增广 | 容易（状态预测/增广） |
| 计算 | 极小 | 极小 | 小（离线增益） | 中（在线 QP） |
| 倒车 | 需特殊处理 | 需特殊处理 | 视模型 | 模型内自然 |

组合方式（回应笔记问题）：**全局规划（Hybrid-A*）+ 局部规划/轨迹生成（TEB）+ 轨迹跟踪（MPC）**是 CV 描述的组合；DWA 直接输出速度指令，一般不再配 MPC 作为"跟踪器"；"TEB + MPC"是"生成器 + 跟踪器"的常见搭配。

---

## 14. 失效模式与故障现象

| 现象 | 可能原因 | 排查 |
|---|---|---|
| **抖动**（转角高频摆动） | 状态噪声/跳变；$R_d$ 太小；参考不连续；线性化切换；求解在约束边界反复切换；延迟 | 看状态是否平滑；看 $\Delta\delta$ 曲线；固定参考做阶跃测试 |
| **跟踪滞后**（总落在轨迹后面） | 时间对齐错位；延迟没补偿；速度参考比车能达到的快 | 查参考时间基准；测 $\tau$；降参考速度 |
| **拐角切内/外扩** | 参考点稀疏（弦误差）；前馈曲率符号/单位错；$\delta$ 映射不准 | 绘制参考与实际轨迹叠图 |
| **求解失败/无解** | 约束冲突；初始状态越界；数值问题 | 状态约束软化；检查初值；增加松弛变量 |
| **求解超时** | $N$ 太大；求解器未热启动；机器负载 | 减 $N$；热启动；查 CPU |
| **倒车时反向打轮** | 线性化的 $v_r$ 符号错；$\delta$ 符号定义不一致 | 倒车单元测试：比较 $\dot\psi$ 符号 |
| **启动瞬间猛打方向** | 初始偏差大；无速率约束 | 加转角速率约束；预对位 |
| **低速时曲率爆炸** | $\kappa=\omega/v$ 在 $v\to0$ 时除零 | 对 $|v|<v_{eps}$ 做保护 |
| **跳变引起的转向突变** | 定位跳变传到参考/状态 | 见 [06 章](./06_EKF与AMCL协同_TF分工与连续状态.md) |

---

## 15. 在 ROS2 里实现 MPC 节点：结构、话题、定时器与时间预算

前面各节是算法。面试官还会问"你的 MPC 在 ROS2 里怎么跑的"，这节把它落到节点结构上。本项目把 MPC 做成一个独立的 ROS2 节点（`mpc_tracker`）；另一种常见形态是把它实现为 Nav2 的控制器插件（继承 `nav2_core::Controller`，由 `controller_server` 以固定频率调用 `computeVelocityCommands`），两者算法相同，差别在于谁负责定时、谁负责与行为树的交互。选独立节点的理由是：TEB 本项目的角色是"局部轨迹生成器"而不是"最终指令生成器"，轨迹由 TEB 单独发布，MPC 独立订阅更直接，也便于单独录包、单独回放测试。

### 15.1 节点的输入输出

| 方向 | 话题 | 类型 | 频率 | QoS |
|---|---|---|---|---|
| 订阅 | `/odometry/filtered` | `nav_msgs/Odometry` | 30 Hz | Reliable，深度 10 |
| 订阅 | `/teb/trajectory` | 带 `time_from_start` 的轨迹消息 | 10 Hz | Reliable，深度 1（只要最新） |
| 订阅 | `/chassis/feedback` | 自定义：实际转角、轮速 | 50 Hz | `SensorDataQoS` |
| 发布 | `/chassis_cmd` | 自定义：速度（mm/s，int16）、转角（0.01°，int16） | 20 Hz | Reliable，深度 1 |
| 发布 | `/mpc/debug` | 预测轨迹、求解耗时、状态标志 | 20 Hz | 默认 |
| 服务 | `/mpc/enable` | `std_srvs/SetBool` | 按需 | 默认 |

其中 `/chassis_cmd` 的单位和格式遵循自查表第 4 项：速度 mm/s、转角 0.01°，带符号的 int16。节点里最容易出错的就是这一步的**单位与符号换算**：MPC 内部用 m/s 与 rad、向左为正，输出时乘 1000、转角乘 $100\times180/\pi$ 并取整、限幅到 int16 范围。

### 15.2 回调与定时器的分工

```text
订阅回调 (多线程回调组: 可重入)             定时器回调 (20 Hz, 互斥回调组)
-------------------------------------       ----------------------------------
on_odom():   写入 latest_state (带锁)         on_timer():
on_traj():   写入 latest_traj  (带锁)            1) 加锁拷贝 latest_state / latest_traj / 已发指令队列
on_feedback(): 写入 latest_fb   (带锁)            2) 延迟前推状态 (120 ms)
                                                 3) 按预测时刻重采样参考 (15 点, 0.1 s)
                                                 4) 线性化 + 填 QP + OSQP 热启动求解
                                                 5) 取第一个输入 -> 限幅 -> 发布
                                                 6) 记录耗时与状态标志
```

关键设计点：（1）**订阅与求解解耦**：回调只做"写最新值"，求解永远在定时器里按固定节拍进行，所以不会因某个话题的突发到达而让求解频率抖动；（2）使用 `MultiThreadedExecutor` 与回调组，保证求解期间订阅回调仍能写入新数据；锁只保护几个拷贝，持有时间远小于 1 ms；（3）轨迹消息的 QoS 深度取 1，MPC 永远只用最新一条，旧轨迹没有意义。

### 15.3 时间预算：50 ms 周期里的每一毫秒

| 环节 | 预算（设计值） | 说明 |
|---|---|---|
| 加锁拷贝与状态前推 | < 0.2 ms | 前推 120 ms 约 12 步 0.01 s 的显式积分 |
| 参考重采样（圆弧插值） | < 0.5 ms | 15 个点，每点一次二分查找+插值 |
| 线性化与填矩阵 | 约 1 ms | $N=15$ 步，每步一个 $4\times4$ 和 $4\times2$ 小矩阵 |
| OSQP 求解 | 2–5 ms（热启动） | 约 90 个变量（15×(4+2)）与数十条约束 |
| 限幅、单位换算、发布 | < 0.3 ms | |
| 合计 | 约 4–7 ms | 约占 50 ms 周期的 10%–15% |

留出这么大余量的原因：（a）周期抖动时不会丢周期；（b）Python 与 C++ 的差别：控制环用 C++（`rclcpp`）实现，若用 Python 写，同一流程会到 10–20 ms，仍在 50 ms 内，但抖动更大，所以 MPC 主循环取 C++，可视化与离线分析用 Python；（c）机载 CPU 同时在跑 EKF、AMCL、局部代价地图、TEB，峰值时单核利用率会升高。

**监控**：节点每个周期记录 `solve_ms`，在 `/mpc/debug` 里发布。设两道阈值：`solve_ms > 10` 记一次警告；`solve_ms > 30` 视为失败，按第 11 节的降级处理（复用上一解的平移序列，最多 3 个周期，再减速停车）。

### 15.4 启动、使能与失效保护

1. 节点启动后处于 **disabled**，输出零速度零转角（保持当前转角的 0.01° 反馈值，而不是回正，避免启动时的突然打方向）；
2. 收到 `/mpc/enable(true)`，且同时满足：`/odometry/filtered` 新鲜（< 100 ms）、协方差迹小于阈值、`/teb/trajectory` 新鲜（< 300 ms）、底盘反馈新鲜（< 100 ms）时，进入 **tracking**；
3. 任何一项失效 → 立刻 **degraded**：速度指令按 $-1.0\ \mathrm{m/s^2}$ 的减速度降到 0（与加速度约束下限一致），转角保持，直到恢复或停车；
4. 看门狗：下位机若 200 ms 收不到指令，自己停车（见 [20 章](./20_工程化实践_频率周期与问题解决.md) 的安全一节）。

### 15.5 参数文件示意

```yaml
mpc_tracker:
  ros__parameters:
    control_rate: 20.0
    horizon_steps: 15
    step_time: 0.1
    q_diag: [10.0, 10.0, 5.0, 1.0]
    r_diag: [0.5, 1.0]
    rd_diag: [0.1, 5.0]
    terminal_scale: 2.0
    delta_max: 0.50
    ddelta_max: 0.6
    v_min: -0.3
    v_max: 1.0
    a_min: -1.0
    a_max: 0.8
    wheelbase: 0.40
    delay_s: 0.12
    solve_warn_ms: 10.0
    solve_fail_ms: 30.0
    max_reuse_cycles: 3
```

这些值与自查表第 19–21 项完全一致；任何一次修改都应同时更新 README 与参数文件。

---

## 16. 面试问答 / 要点速记 / 自测题 / 链接

### 面试问答

**Q1. Describe your MPC tracking controller.**

> It's a receding-horizon controller on a car-like kinematic model. The state is position, heading and speed, the inputs are acceleration and front steering angle. Around the reference trajectory I linearise and discretise the model, and write the cost as weighted tracking errors, control effort and control change, plus a terminal term. Constraints cover steering angle and rate, speed and acceleration limits. At each cycle I solve the resulting QP, apply only the first input, and solve again with the new state.
> 本项目取值约：$N=15$、步长 0.1 s（预测 1.5 s）；$Q=\mathrm{diag}(10,10,5,1)$，$R=\mathrm{diag}(0.5,1)$，$R_d=\mathrm{diag}(0.1,5)$，终端权重取 $2Q$；约束为 $|\delta|\le0.50$ rad、$|\dot\delta|\le0.6$ rad/s、$-0.3\le v\le1.0$ m/s、$-1.0\le a\le0.8$ m/s²；求解器 OSQP，20 Hz。

*追问*：Why only the first input? —— 反馈来自重复求解；模型有误差，后面的输入是基于预测，不必执行。

**Q2. How are the control inputs chosen? Do you provide candidate values?**

> No. The inputs are the decision variables. I give the solver the model, the weights, the constraints, the current state and the reference. It returns the sequence of accelerations and steering angles that minimises the cost while satisfying the constraints. The previous solution, shifted by one step, is only used as a warm start.

**Q3. How did you align the reference trajectory in time, and handle delay?**

> The local trajectory carries timestamps, so I resample it at the MPC prediction times, using arc or heading-based interpolation rather than straight lines when points are sparse. For delay, I estimate the total latency, predict the vehicle state forward by that time with the commands already sent, and start the optimisation from the predicted state with the reference shifted accordingly.
> 本项目做了延迟补偿：测得指令到车体响应的总延迟约 120 ms（串口与下位机约 20 ms，舵机与电机响应约 100 ms），用已发出的指令把状态向前预测 0.12 s 后再起算，参考按时间戳对齐，点稀疏时用圆弧插值。

**Q4. What is the terminal cost for?**

> With a finite horizon the optimiser doesn't care what happens after the last step. A terminal cost, ideally the LQR Riccati solution, makes the finite-horizon cost approximate the infinite-horizon one and helps stability. In practice, I use a larger multiple of the stage weights and check it in simulation and on the vehicle.

**Q5. Why MPC rather than Pure Pursuit?**

> Pure Pursuit is simple and has no constraints or prediction; limits and delay are handled outside. MPC puts the steering limits, rates, speed bounds and the vehicle model into one optimisation, and looks ahead, which gives smoother tracking and makes delay compensation straightforward. The price is computation and tuning.

### 本章要点速记

1. 滚动时域：每周期解 $N$ 个动作，执行第 1 个；$N$ 个动作对应 $N+1$ 个状态（含测量的 $\mathbf x_0$）。
2. 模型：$(x,y,\psi,v)$，输入 $(a,\delta)$；$\dot\psi=v\tan\delta/L$。
3. 线性化：$A$ 的 $(1,3)(2,3)(3,4)$ 项，$B_{\psi\delta}=v_r/(L\cos^2\delta_r)$；离散 $A_d=I+A\Delta t$。
4. 代价：$Q$（跟踪）、$R$（用力）、$R_d$（平顺）、$Q_f$（终端，可取 Riccati 解）；权重用 $1/\text{可接受值}^2$ 起步。
5. 约束：输入界与变化率（硬），状态界（软，松弛变量）。
6. 决策变量是 $U$，优化器自己选，不是候选挑选；上一解仅作热启动。
7. 参考要按时间对齐并插值，稀疏点用圆弧插值，前馈 $\delta_r=\arctan(L\kappa_r)$。
8. 延迟 → 状态预测补偿；0.15 s 在 1 m/s 下是 0.15 m、4.3°。
9. 实机必须有求解失败回退、输入限幅、有效性检查与看门狗。

### 自测题

1. $N=8$ 时有几个动作、几个状态、其中几个是预测量？（答：8 个动作；9 个状态；8 个预测量，$\mathbf x_0$ 是测量值。）
2. 在第 7.2 节中把 $r$ 改为 10，$u_0,u_1$ 大约如何变化？（提示：$H=\begin{bmatrix}10.4&0.2\\0.2&10.2\end{bmatrix}$，解得 $u_0\approx-0.0095$，$u_1\approx-0.0047$；动作约缩小到原来的四分之一，响应更慢。）
3. $L=0.5$、$v_r=2$、$\delta_r=0$，$B$ 中 $\psi$ 对 $\delta$ 的元素是多少？（答：$2/0.5=4$。）
4. 为什么状态约束常用软约束、输入约束常用硬约束？
5. 若 $\tau=0.2$ s、控制周期 0.05 s，需要缓存几步命令？（答：4。）

### 相关链接

- 上一章：[09 TEB 局部轨迹优化](./09_TEB局部轨迹优化.md)　下一章：[11 联调与故障案例](./11_联调与故障案例.md)
- 返回：[README](./README.md)
- 基础：[../../02_基础知识/05_控制理论/05_MPC模型预测控制.md](../../02_基础知识/05_控制理论/05_MPC模型预测控制.md)
- 学习笔记：[../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md](../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md)
