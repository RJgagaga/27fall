# LQR 原理与按腿长增益调度：从 Riccati 方程到 K(L)

> **一句话定位**：LQR 不是"状态乘一张表"。它是**两步**：离线用模型 $(A,B)$ 和权重 $(Q,R)$ 解 Riccati 方程得到最优增益 $K$；在线只做 $\mathbf u=-K\mathbf s$。P7 的特色是第一步对一串腿长各做一次，再把 $K$ 的 12 个元素各拟合成腿长的多项式，在线按当前 $L$ 算增益——这就是"增益调度 LQR"。

**本章学完你能：**

- 说清 LQR 要解决什么问题，$Q$、$R$ 各代表什么，怎样从"可接受误差"选出初值；
- 从最优性条件一步步推出代数 Riccati 方程与 $K=R^{-1}B^\top P$；
- 手算一个标量例子和一个双积分器例子，验证 Riccati 解；
- 读懂六状态双输出的 $2\times6$ 增益矩阵 $K$：每一行每一列是什么意思、符号为什么是这样；
- 讲清楚"为什么要按腿长调度"，并用数字展示固定增益的退化与失稳；
- 写出按腿长的多项式拟合流程与在线计算代码（教学示例），说明范围保护；
- 对照课程第八、九、十一课的真实实现。

**前置知识**：

- 状态空间与可控性：[状态空间、可控可观、稳定性](../../02_基础知识/04_控制理论/03_状态空间_可控可观_稳定性.md)
- LQR 通识：[LQR 最优控制](../../02_基础知识/04_控制理论/04_LQR最优控制.md)
- 增益调度与饱和：[前馈、增益调度、限幅与饱和](../../02_基础知识/04_控制理论/07_前馈_增益调度_限幅与饱和.md)
- 线性代数（矩阵、特征值）：[线性代数](../../02_基础知识/01_数学/01_线性代数.md)
- 上一章模型：[04_动力学建模与状态方程.md](./04_动力学建模与状态方程.md)

**目录**

1. LQR 要解决什么问题
2. 代价函数与 Q、R 的含义
3. 推导：从最优性到 Riccati 方程
4. 两个手算例子
5. 课程第八课：四状态、单输出 LQR 的数字
6. 六状态、双输出 LQR：K 是 2×6，怎么读
7. Q、R 怎么调（带数字的灵敏度）
8. 为什么要按腿长调度
9. 增益调度的完整流程与多项式拟合
10. 在线实现：K(L) 与 u = −K(s − s_ref)
11. 离散化、延迟与"冻结腿长"的局限
12. 对照课程源码
13. 常见错误与故障现象
14. 面试怎么问、怎么答
15. 要点速记、自测题、相关链接

---

## 1. LQR 要解决什么问题

线性系统 $\dot{\mathbf s}=A\mathbf s+B\mathbf u$ 不稳定（倒立摆）。我们想用**状态反馈**稳定它：$\mathbf u=-K\mathbf s$，使闭环 $\dot{\mathbf s}=(A-BK)\mathbf s$ 的所有特征值实部为负。

问题是 $K$ 有无穷多种选法，怎么选才"好"？

- 太小：恢复慢，扰动大时会倒；
- 太大：控制量大（力矩饱和）、对噪声和延迟敏感、振荡。

LQR（Linear Quadratic Regulator，线性二次型调节器）的思想：**把"状态误差大"和"控制量大"用一个数（代价）衡量，选使总代价最小的 $K$**。它把"调增益"变成了"调权重"，而且保证（在可控的前提下）得到一个**稳定的**闭环，并有良好的鲁棒裕度（理论上具有至少 60° 相位裕度和无穷增益裕度，对连续时间、权重选取合理的情形）。

---

## 2. 代价函数与 Q、R 的含义

$$
J=\int_0^\infty\big(\mathbf s^\top Q\,\mathbf s+\mathbf u^\top R\,\mathbf u\big)\,dt
$$

- $\mathbf s$：状态（相对平衡点的偏差）。$\mathbf u$：控制量。
- $Q\succeq0$（半正定，通常取对角）：**状态惩罚**。$Q_{ii}$ 越大，第 $i$ 个状态的偏差越"不能容忍"，控制器会更努力把它压小。
- $R\succ0$（正定，通常取对角）：**控制量惩罚**。$R_{jj}$ 越大，第 $j$ 个输入越"贵"，控制器越节省用力。
- **只有 $Q$ 与 $R$ 的相对大小重要**：$Q,R$ 同时乘以同一个常数，$K$ 不变。

### 2.1 初值怎么选：Bryson 经验法则

$$
Q_{ii}=\frac{1}{(\text{第 }i\text{ 个状态可接受的最大偏差})^2},\qquad
R_{jj}=\frac{1}{(\text{第 }j\text{ 个输入允许的最大值})^2}
$$

**示例（教学数值）**：本章六状态例子用

$$
Q=\operatorname{diag}(100,\ 1,\ 500,\ 100,\ 5000,\ 1),\qquad R=\operatorname{diag}(1,\ 0.25)
$$

按 Bryson 反推，相当于：腿角容许 0.1 rad，腿角速度 1 rad/s，位置 0.045 m，速度 0.1 m/s，机身俯仰 0.014 rad（约 0.8°），俯仰角速度 1 rad/s；轮力矩容许 1 N·m，髋力矩容许 2 N·m。**俯仰权重最大**，因为机身俯仰是最危险的量。

本项目取值与上式同量级：$Q$ 对角线从 1 到 5000（俯仰最大），$R$ 对角线为 1 和 0.25；实车上主要只微调 $Q_\phi$ 与速度项权重（见 README 自查表）。

---

## 3. 推导：从最优性到 Riccati 方程

**目标**：找 $\mathbf u(t)$ 使 $J$ 最小。

**第 1 步：价值函数。** 设从状态 $\mathbf s$ 出发的最优代价为 $V(\mathbf s)$。线性二次问题的价值函数是二次型：

$$
V(\mathbf s)=\mathbf s^\top P\,\mathbf s,\qquad P=P^\top\succ0
$$

**第 2 步：最优性方程（HJB）。** 在每一刻选择使"瞬时代价 + 价值变化率"最小的控制：

$$
0=\min_{\mathbf u}\Big[\mathbf s^\top Q\mathbf s+\mathbf u^\top R\mathbf u+\nabla V^\top(A\mathbf s+B\mathbf u)\Big],\qquad \nabla V=2P\mathbf s
$$

**第 3 步：对 $\mathbf u$ 求最小。** 括号对 $\mathbf u$ 求导令零：

$$
2R\mathbf u+2B^\top P\mathbf s=0\ \Rightarrow\ \boxed{\mathbf u^*=-R^{-1}B^\top P\,\mathbf s=-K\mathbf s},\quad K=R^{-1}B^\top P
$$

**第 4 步：代回消去 $\mathbf u$。**

$$
\mathbf u^{*\top}R\mathbf u^*=\mathbf s^\top PBR^{-1}B^\top P\mathbf s,\qquad
2\mathbf s^\top P(A\mathbf s+B\mathbf u^*)=\mathbf s^\top(PA+A^\top P)\mathbf s-2\mathbf s^\top PBR^{-1}B^\top P\mathbf s
$$

相加后对任意 $\mathbf s$ 为零，得到**代数 Riccati 方程（ARE）**：

$$
\boxed{A^\top P+PA-PBR^{-1}B^\top P+Q=0}
$$

**第 5 步：稳定性的证据。** 取 $V=\mathbf s^\top P\mathbf s$ 为李雅普诺夫函数，闭环下

$$
\dot V=-\mathbf s^\top Q\mathbf s-\mathbf u^{*\top}R\mathbf u^*<0
$$

所以闭环渐近稳定。前提：$(A,B)$ 可镇定（本章系统可控，秩为 6），$(Q^{1/2},A)$ 可检测（对角 $Q$ 的相应项为正即可）。

**流程图：**

```text
模型 (A, B)  ─┐
权重 (Q, R)  ─┴─► 解 ARE → P ─► K = R⁻¹ BᵀP   （离线，MATLAB lqr / scipy.solve_continuous_are）
                                      │
状态 s(t) ────────────────────────────┴──► u = −K s   （在线，一次矩阵乘法）
```

---

## 4. 两个手算例子

### 4.1 标量系统（最小可解例子）

$\dot s=a\,s+b\,u$，$a=1$（不稳定），$b=1$，$q=1$，$r=1$。

ARE 退化为一个方程：

$$
2ap-\frac{b^2p^2}{r}+q=0\ \Rightarrow\ 2p-p^2+1=0\ \Rightarrow\ p=1+\sqrt2\approx2.414
$$

$$
K=\frac{bp}{r}=2.414,\qquad \text{闭环极点}=a-bK=1-2.414=-1.414=-\sqrt2
$$

**一般结果**：闭环极点 $=-\sqrt{a^2+b^2q/r}$。把 $r$ 改成 0.1（控制量变便宜）：

$$
K=a+\sqrt{a^2+b^2q/r}=1+\sqrt{11}=4.317,\qquad \text{极点}=-3.317
$$

结论：**$R$ 越小（控制越便宜）→ $K$ 越大 → 闭环越快**。

### 4.2 双积分器

$\dot{\mathbf s}=\begin{bmatrix}0&1\\0&0\end{bmatrix}\mathbf s+\begin{bmatrix}0\\1\end{bmatrix}u$，$Q=I$，$R=1$。

设 $P=\begin{bmatrix}p_1&p_2\\p_2&p_3\end{bmatrix}$。算出：

$$
A^\top P=\begin{bmatrix}0&0\\p_1&p_2\end{bmatrix},\quad PA=\begin{bmatrix}0&p_1\\0&p_2\end{bmatrix},\quad PBB^\top P=\begin{bmatrix}p_2^2&p_2p_3\\p_2p_3&p_3^2\end{bmatrix}
$$

ARE 的三个独立分量：

| 位置 | 方程 | 结果 |
|---|---|---|
| (1,1) | $-p_2^2+1=0$ | $p_2=1$ |
| (1,2) | $p_1-p_2p_3=0$ | $p_1=p_3$ |
| (2,2) | $2p_2-p_3^2+1=0$ | $p_3=\sqrt3$ |

所以 $P=\begin{bmatrix}\sqrt3&1\\1&\sqrt3\end{bmatrix}$，$K=B^\top P=[1,\ \sqrt3]$，闭环矩阵 $\begin{bmatrix}0&1\\-1&-\sqrt3\end{bmatrix}$ 的特征方程 $\lambda^2+\sqrt3\lambda+1=0$，极点 $-0.866\pm0.5j$（阻尼比 0.866）。

这是理解"LQR 给出的反馈 = 位置增益 1，速度增益 $\sqrt3$"的最小例子。

---

## 5. 课程第八课：四状态、单输出 LQR 的数字

模型：上一章 3.3 节的四状态"轮 + 摆杆"；输入 $\tau$ 一个；$K$ 是 $1\times4$。

MATLAB 脚本 `wheel_control.m`：$m=6.25,\ M=3.7,\ h=0.2156,\ r=0.07,\ g=9.8$，`Q=diag([1 1 1 1]); R=1; K=lqr(A,B,Q,R)`。

我用同一模型（课程写法的 B）在 Python 里复算，得到：

| 摆杆质心高 $h$（m） | K（$\theta,\dot\theta,x,\dot x$） |
|---|---|
| 0.085 | [−14.71, −2.12, **−1.00**, −4.48] |
| 0.130 | [−14.91, −2.60, **−1.00**, −4.54] |
| 0.2156 | [−15.24, −3.43, **−1.00**, −4.62] |

与课程 `LQRfitting.m` 中手填的三行 $K$（−14.6115 −1.9675 −1 −4.3543 等）**量级一致**。有一个很清晰的现象：**位置增益恰好是 −1.0000**（$Q_x=1,R=1$ 时数值上恒为 $-\sqrt{Q_x/R}$），这与课程表里的 −1.0 一致，是对"模型和求解流程与课程相同"的一个旁证。

### 5.1 课程里的"手工调整"

`LQRfitting.m` 用三个腿高 `hs=[140 215 290]` 对应的三行 $K$ 做二次多项式拟合：

```matlab
Ks = [ -14.6115  -1.9675  -1.0*0   -4.3543;
       -14.8728  -2.5894  -1.0*0   -4.3543;
       -14.8198*0.5  -3.1558  -0.7071*0  -3.7071 ];
poly_order = 2;  P(i,:) = polyfit(hs, Ks(:,i), poly_order);
```

你能看到三件事：

1. **位置增益被乘了 0**（`-1.0*0`）：课程里把位置反馈关掉了（只做速度/姿态稳定，不要求回到原位）；
2. 第三行 $K_1$ 被乘了 0.5：手工缩小，说明"纯 LQR 输出"还要被实机整定；
3. 拟合是**逐元素**的：$K_1\ldots K_4$ 各一条二次多项式，输入是腿高 $h$。

这就是"LQR 设计 + 多项式调度 + 实机整定"的一个最小实现，和你 CV 里的 P7 是同一套思想，只是 P7 的维度更高（六状态、两个输出，按**虚拟腿长**调度）。

---

## 6. 六状态、双输出 LQR：K 是 2×6，怎么读

状态 $\mathbf s=[\theta,\dot\theta,x,\dot x,\phi,\dot\phi]^\top$，输入 $\mathbf u=[T,T_p]^\top$。$K\in\mathbb R^{2\times6}$：

$$
\begin{bmatrix}T\\T_p\end{bmatrix}=-\begin{bmatrix}k_{11}&\cdots&k_{16}\\k_{21}&\cdots&k_{26}\end{bmatrix}\begin{bmatrix}\theta\\\dot\theta\\x\\\dot x\\\phi\\\dot\phi\end{bmatrix}
$$

- 第 1 行 6 个增益共同决定**轮毂力矩 $T$**；
- 第 2 行 6 个增益共同决定**髋部力矩 $T_p$**；
- 每个输入都看全部 6 个状态——这是**多输入 LQR**的特点：两个输出是**协同**的，不是"轮子管一部分、髋管另一部分"。

### 6.1 教学数值：L=0.2 m 的 K

用第 4 章的教学参数和上面的 $Q,R$，解 ARE 得到（单位：$T$ 的行为 N·m 每状态单位）：

$$
K(0.2)=\begin{bmatrix}
-35.09&-5.52&-20.88&-17.51&-32.07&-2.50\\
-24.62&-3.87&-16.01&-12.95&129.74&2.80
\end{bmatrix}
$$

闭环极点：$-239.8,\ -42.9\pm30.6j,\ -7.81,\ -2.84\pm0.17j$，全部在左半平面。

### 6.2 逐个元素读符号（约定 S：前倾为正）

- $k_{11}=-35.09$，$T=-k\,\theta=+35.09\,\theta$：腿角 $\theta>0$（髋在轮前方，身体在轮前面）→ $T>0$，轮向前加速去"追"身体。✓ 符合倒立摆"往倾倒方向开"的直觉。
- $k_{14}=-17.5$，所以 $T=-k_{14}(\dot x-v_{set})=+17.5\,(\dot x-v_{set})$：**速度比指令快（误差为正）时，$T>0$，轮子继续向前加速**。

> **这看起来违反直觉**：车跑快了，不是应该减速吗？原因是倒立摆的**非最小相位**特性：要让速度降下来，必须先让重心落到轮子后面（身体相对轮向后倒），而身体向后倒恰恰是靠"先把轮子往前冲"实现的（上一章 $B_{21}<0$：$T>0$ 使 $\theta$ 减小）。反过来，想从静止加速到 $v_{set}>0$ 时，误差 $\dot x-v_{set}<0$，$T$ 先**为负**（轮子先向后退，让身体前倾），再变正。LQR 自动找到了这种"先反向、再纠正"的策略。你笔记里没看懂的"为什么符号不像普通 PID"，答案就在这里。第 13 章仿真数字：速度阶跃 0.5 m/s 的第一拍 $T\approx-8.8$ N·m。

- $k_{25}=+129.74$，$T_p=-129.74\,\phi$：机身前倾（$\phi>0$）→ $T_p<0$，髋力矩把机身往后拉回。强度：每 1° 俯仰误差需要约 2.26 N·m 的髋力矩（$129.74\times\pi/180$）。**这个比例决定了控制器的线性工作区**：若髋电机力矩限幅 $X$ N·m，则线性区约为 $|\phi|<X/129.74$ rad；超出就饱和（第 11 章）。
- $k_{21},k_{23}$ 等：$T_p$ 对腿角、位置的响应，一般比对 $\phi$ 的小。

### 6.3 随腿长变化的 K

| L（m） | 第 1 行（T） | 第 2 行（Tp） |
|---|---|---|
| 0.10 | −21.28, −2.65, −18.70, −15.29, −47.03, −2.95 | −25.69, −3.30, −24.53, −19.46, 110.92, 1.70 |
| 0.20 | −35.09, −5.52, −20.88, −17.51, −32.07, −2.50 | −24.62, −3.87, −16.01, −12.95, 129.74, 2.80 |
| 0.30 | −46.81, −8.67, −21.61, −19.01, −24.15, −2.24 | −23.12, −4.18, −11.48, −9.70, 136.12, 3.29 |

规律：$k_{11}$（腿角 → 轮矩）随腿长近似线性增长（−21 → −35 → −47）；$k_{15}$（俯仰 → 轮矩）则缩小；$k_{25}$ 从 111 增至 136。**十二个元素以各自不同的方式随 $L$ 变化，没有一个统一缩放系数**——所以必须逐元素拟合。

---

## 7. Q、R 怎么调（带数字的灵敏度）

在 $L=0.2$ m，基准 $Q=\operatorname{diag}(100,1,500,100,5000,1)$，$R=\operatorname{diag}(1,0.25)$，改变一个权重、观察结果。"峰值"指初始俯仰 $\phi_0=0.05$ rad 的线性仿真里的最大力矩。

| 改动 | $k_{15}$ | $k_{25}$ | 最慢极点实部 | 峰值 $T_p$（N·m） | 说明 |
|---|---|---|---|---|---|
| 基准 | −32.1 | 129.7 | −2.84 | 6.5 | |
| $Q_\phi\times0.1$ | −16.7 | 36.7 | −2.84 | 1.8 | 姿态变松，力矩需求骤降 |
| $Q_\phi\times10$ | −62.3 | 432.3 | −2.84 | 21.6 | 姿态变硬，力矩需求 3 倍 |
| $Q_v\times10$ | −50.1 | 109.9 | **−0.71** | 5.5 | 速度项被压得太狠，最慢极点变慢 |
| $Q_x\times0.1$ | −30.3 | 131.1 | **−0.71** | 6.6 | 不管位置了，回位变慢 |
| $Q_x\times10$ | −37.7 | 125.2 | −4.54 | 6.3 | 回位更快，但会拉动姿态 |
| $R_T$ 取 4（轮矩更贵） | −21.1 | — | −2.72 | — | 轮子反应变小 |

**经验：**

1. 想让机身更"硬"（俯仰误差更小）→ 增大 $Q_\phi$，代价是**髋力矩需求按比例上升**——这与电机限幅直接冲突；
2. 想让速度跟得更紧 → 增大 $Q_v$，但要防止最慢极点被拖慢；
3. 位置项 $Q_x$ 大会让机器人"拼命回原点"，在不平路面容易引发抖动；很多实现把它取小甚至取零（课程里的 $K_3$ 被置 0）；
4. 一切调整以**实机力矩是否饱和、是否振荡**为准，而不是以仿真为准（第 11 章）。

---

## 8. 为什么要按腿长调度

### 8.1 动机（来自第 4 章模型）

腿长变化 → $A(L),B(L)$ 变化：不稳定极点按约 $1/\sqrt L$ 移动，输入增益 $B_{21}$ 变化 4 倍以上。最优的 $K$ 自然随 $L$ 变化。

### 8.2 数字演示：固定增益在不同腿长下的表现

用 $L=0.2$ m 设计出的 $K(0.2)$ 固定不变，放到不同腿长的真实线性模型上，看闭环最慢极点的实部（越负越好，正数 = 失稳）：

| 腿长 L（m） | 固定 $K(0.2)$ | 按腿长重新设计的 $K(L)$ |
|---|---|---|
| 0.10 | −1.57 | −2.69 |
| 0.15 | −2.48 | −2.82 |
| 0.20 | −2.84 | −2.84 |
| 0.25 | −2.15 | −2.80 |
| 0.30 | −2.00 | −2.77 |
| 0.40 | **+36.2（失稳）** | −2.68 |
| 0.50 | **+110.7（失稳）** | −2.59 |

解读（诚实地看）：

- 在 0.10–0.30 m 内，固定增益在这个简化模型里**仍然稳定**，但收敛速度退化（0.10 m 时最慢极点从 −2.69 退到 −1.57，慢了约 40%）；
- 腿长范围一大（0.4 m 以上），固定增益**直接失稳**；
- 真实机器人还有延迟、饱和、模型误差，裕度比这个理想模型小得多，所以**实机上固定增益的可用范围更窄**。

> **面试要点（"Why must the gains be scheduled by leg length?"）**：因为被控对象随腿长变化——惯量、重心高度、力矩杠杆都在变，线性模型的 $A,B$ 都是 $L$ 的函数。固定增益只在设计腿长附近最优；离开后性能下降，范围大了会失稳。增益调度让每个腿长都用接近最优的增益。

### 8.3 为什么不是"每个周期现场解 Riccati"

解 ARE 需要迭代（Schur 分解或牛顿法），在 STM32 上每个周期做不划算，也不可预测。所以：**离线做，在线查**。

---

## 9. 增益调度的完整流程与多项式拟合

### 9.1 流程

```text
1. 选腿长网格：L1 < L2 < … < LN（覆盖工作范围，加余量）
2. 对每个 Li：建 A(Li), B(Li)（第 4 章），解 ARE，得 K(Li)（2×6）
3. 对 K 的 12 个元素逐个拟合：k_ij(L) ≈ c3 L³ + c2 L² + c1 L + c0
4. 检查：拟合误差；用拟合后的 K 在密集 L 上验证闭环稳定
5. 把 12×4 = 48 个系数写进代码；运行时按当前 L0 计算
6. 实车整定：在 K 上乘缩放、调 Q/R 重新离线算，直到表现满意
```

### 9.2 本章教学例子的拟合结果

在 $L\in[0.10,0.30]$ 取 9 个等距点，三次多项式拟合（**教学示例，非项目系数**）。系数从高次到低次：$k(L)=c_3L^3+c_2L^2+c_1L+c_0$。

| 元素 | $c_3$ | $c_2$ | $c_1$ | $c_0$ |
|---|---|---|---|---|
| $k_{11}$ | −164.506 | 202.556 | −187.303 | −4.412 |
| $k_{12}$ | 29.550 | −31.889 | −21.189 | −0.244 |
| $k_{13}$ | −293.765 | 248.978 | −75.980 | −13.302 |
| $k_{14}$ | −90.615 | 91.022 | −43.255 | −11.778 |
| $k_{15}$ | 832.787 | −851.896 | 346.907 | −74.027 |
| $k_{16}$ | 17.673 | −20.009 | 9.248 | −3.690 |
| $k_{21}$ | −302.579 | 203.932 | −29.375 | −24.505 |
| $k_{22}$ | −51.166 | 43.784 | −15.278 | −2.158 |
| $k_{23}$ | 420.119 | −451.538 | 191.294 | −39.572 |
| $k_{24}$ | 484.742 | −453.911 | 167.302 | −32.121 |
| $k_{25}$ | 2665.193 | −2223.281 | 668.763 | 63.706 |
| $k_{26}$ | 103.669 | −93.058 | 31.662 | −0.634 |

**拟合质量**：三次拟合的最大绝对误差 0.14，最大相对误差 0.13%（相对于该元素最大值）；二次拟合最大绝对误差 0.79（0.9%）。在 $L\in[0.1,0.3]$ 的 41 个点上用拟合后的 $K$ 检查，闭环最慢极点实部最大为 −2.69，全部稳定。

**手算：$k_{11}(0.2)$**

$$
\begin{aligned}
k_{11}(0.2)&=-164.506(0.008)+202.556(0.04)-187.303(0.2)-4.412\\
&=-1.316+8.102-37.461-4.412=-35.087
\end{aligned}
$$

与直接解 ARE 的 $-35.086$ 吻合。

**为什么用多项式而不是查表+线性插值：** 多项式光滑、占内存小（48 个 float）、计算量固定；插值需要存网格并做区间搜索，且在节点处导数不连续（对 $L$ 变化时的增益跳变更敏感）。两种都常见，本项目采用多项式：三次、12 个增益各 4 个系数共 48 个、拟合区间 $L_0\in[0.10,0.30]$ m。学习笔记里记录的参考实现是"12 个增益、每个 4 个系数（三次）共 48 个"，与上面一致。

### 9.3 范围保护：多项式只在拟合区间内可信

在区间外做外推时误差迅速增大。例如元素 $k_{15}$ 在 $L=0.5$ 时，三次拟合给 −9.45，而真实最优增益是 −16.48。因此在线代码**必须把 $L_0$ 限制在拟合区间内**（clamp），并对 $L_0$ 做低通/限速，避免 FK 异常值把增益推到荒谬的位置（第 11 章 F13）。

---

## 10. 在线实现：K(L) 与 u = −K(s − s_ref)

### 10.1 教学示例代码（C，非项目源码）

```c
#define NS 6
#define NU 2
#define L_MIN 0.10f   /* 拟合区间下限（示例） */
#define L_MAX 0.30f   /* 拟合区间上限（示例） */

/* 每个元素的三次多项式系数：{c3, c2, c1, c0}（教学示例，来自本章 9.2 节的表） */
static const float kc[NU][NS][4] = {
  {{-164.506f, 202.556f,-187.303f,  -4.412f},
   {  29.550f, -31.889f, -21.189f,  -0.244f},
   {-293.765f, 248.978f, -75.980f, -13.302f},
   { -90.615f,  91.022f, -43.255f, -11.778f},
   { 832.787f,-851.896f, 346.907f, -74.027f},
   {  17.673f, -20.009f,   9.248f,  -3.690f}},
  {{-302.579f, 203.932f, -29.375f, -24.505f},
   { -51.166f,  43.784f, -15.278f,  -2.158f},
   { 420.119f,-451.538f, 191.294f, -39.572f},
   { 484.742f,-453.911f, 167.302f, -32.121f},
   {2665.193f,-2223.281f,668.763f,  63.706f},
   { 103.669f, -93.058f,  31.662f,  -0.634f}}
};

static inline float poly3(const float c[4], float L) {
  return ((c[0] * L + c[1]) * L + c[2]) * L + c[3];   /* Horner 法 */
}

void lqr_step(const float s[NS], const float sref[NS], float L0,
              float limT, float limTp, float *T, float *Tp) {
  float L = L0 < L_MIN ? L_MIN : (L0 > L_MAX ? L_MAX : L0);  /* 范围保护 */
  float u[NU] = {0, 0};
  for (int i = 0; i < NU; ++i)
    for (int j = 0; j < NS; ++j)
      u[i] -= poly3(kc[i][j], L) * (s[j] - sref[j]);          /* u = -K (s - sref) */
  *T  = u[0] > limT  ? limT  : (u[0] < -limT  ? -limT  : u[0]);
  *Tp = u[1] > limTp ? limTp : (u[1] < -limTp ? -limTp : u[1]);
}
```

要点：

- 每个周期 $12$ 次三次多项式（各 3 次乘加）+ $12$ 次乘加：几百次浮点运算，STM32H7 带 FPU 完全没有压力；
- Horner 法减少乘法并提高数值稳定性；
- 先限幅 $L$、后算增益、最后限幅输出，三道保护缺一不可。

### 10.2 参考量 $\mathbf s_{ref}$

$\mathbf s_{ref}=[0,\ 0,\ x_{set},\ v_{set},\ 0,\ 0]^\top$：

- 平衡点 $\theta,\phi$ 目标为零（或实机标定后的机械零位偏置）；
- $v_{set}$ 来自遥控/上层指令，**必须经过斜坡限幅**。教学模型里的数字：速度指令从 0 阶跃到 0.5 m/s，瞬间的轮矩需求 $T=17.5\times0.5\approx8.8$ N·m；同样的指令用 1 s 斜坡，峰值只有 0.35 N·m。**斜坡是对"力矩限幅"最便宜的保护。**
- $x_{set}$ 由 $v_{set}$ 积分得到，静止或离地时要复位（第 8、9 章），否则位置误差会不断累积。

### 10.3 符号

标准写法是 $u=-K\mathbf s$。课程代码里写成 `-(K1*pitch + K2*gyro + K3*pos + K4*vel)`，$K$ 里本身带负号，两种写法等价。**不要在已经包含符号的 $K$ 外面再多加一个负号**（笔记第 17 组曾提到这一点）；判断对错只看"小扰动是否被拉回"（第 3 章 7 节）。

---

## 11. 离散化、延迟与"冻结腿长"的局限

- **连续 vs 离散**：上面的 $K$ 按连续时间设计。控制周期 $T_s$ 远小于系统最快时间常数（本例最快闭环极点 −240 rad/s，时间常数约 4 ms；若 $T_s=1$ ms，勉强可用，$T_s=5$ ms 就需要用离散 LQR `dlqr`）。工程上常见做法：用 ZOH 离散化后在**离散域**重新解 DARE。
- **延迟**：电机 + 通信 + 计算延迟会减少相位裕度。表现为高增益下的高频振荡，对策是降低含速度项的增益、提高控制频率、降低 $Q$ 的"速度权重"。
- **冻结腿长假设**：每个 $K(L)$ 是在"腿长固定"的前提下设计的。腿长**快速变化**时，$A(L(t))$ 随时间变化，冻结时间的稳定性分析不再严格成立。所以工程上：**限制腿长变化速率**（第 7 章），并在实车上验证腿长伸缩过程中的稳定性。这也是 CV 里写"while leg length changed"的难点所在。

---

## 12. 对照课程源码

| 内容 | 位置 | 说明 |
|---|---|---|
| 四状态 LQR 设计 | 第八课 `wheel_control.m` | `lqr(A,B,Q,R)`，$Q=I,R=1$ |
| 手填 K 与多项式拟合 | 第九课 `LQRfitting.m` | 二次多项式；自动生成 C 代码 `calc_K1…K4(h)` |
| 第九课部署 | `src/Robot/robot.cpp: calc_K*` 与 `wheel_control()` | `calc_K1(h)=0.000687h²−0.247225h+6.542786` 等；**但 `wheel_control` 里实际用的是写死的常数 K1=−7.41, K2=−3.16, K3=−0.71, K4=−3.71**（最大腿高那一组），多项式函数在该课里没被调用 |
| 第十一课部署 | `src/robot.cpp: wheel_control()` | 调用 `calc_K1…K4(RightPosFK->y)`，**用腿高 $y$（mm）调度**；$K_3=0$ |
| 轮目标表达式 | 同上 | `-(K1*pitch*π/180 + K2*gyroY*π/180 + K3*position + K4*vel) - 0.3*steering - 0.1*forwardBackward`，限幅 ±`speed_limit`=3 |

第十一课四个多项式在几个腿高下的值（由源码公式计算）：

| 腿高 y（mm） | $K_1$ | $K_2$ | $K_3$ | $K_4$ |
|---|---|---|---|---|
| 110 | −13.02 | −0.83 | 0 | −4.33 |
| 140 | −14.37 | −1.31 | 0 | −4.39 |
| 215 | −12.60 | −2.15 | 0 | −4.48 |
| 290 | −3.50 | −2.49 | 0 | −4.51 |

课程用**腿高 $y$（mm）**调度四状态的单输出轮控制；你的 P7 是**虚拟腿长 $L_0$** 调度六状态、两个输出（轮毂力矩 $T$ 与髋部力矩 $T_p$）。这是**课程与你项目的口径差异**，面试以 CV 为准（README 口径备忘）。

---

## 13. 常见错误与故障现象

| 现象 | 原因 | 处理 |
|---|---|---|
| 一开控制就往一个方向冲 | 某个状态符号反（尤其 $\theta$、$\phi$、$\dot x$） | 第 3 章 7 节核对清单 |
| 腿长一变就抖 | 腿长参与调度但 $L_0$ 噪声大，增益跳 | 对 $L_0$ 低通+限速；检查 FK |
| 机身偶尔猛地一"抽" | $T_p$ 饱和 → 有效增益骤降 | 查限幅值与线性区（第 6.2 节） |
| 低速抖、静止时蠕动 | 位置项权重大、速度估计有偏 | 减小 $Q_x$ 或位置增益；改进速度估计 |
| 腿长在范围外时失稳 | 多项式外推 | clamp $L_0$；扩大拟合区间并重做 |
| 仿真很稳实机很差 | 模型误差、延迟、饱和 | 降增益、提频、实车整定 |

---

## 14. 面试怎么问、怎么答

**Q1. Explain LQR. How does it work and how did you use it?**

> LQR designs a state feedback gain by minimizing a quadratic cost: the integral of the state error weighted by Q plus the control effort weighted by R. For a linear model x-dot equals A x plus B u, I solve the algebraic Riccati equation offline to get P, and the optimal gain is K equals R inverse B transpose P. Online, the control is just u equals minus K times the state error. In my project the state has six elements and the input has two, a wheel-hub torque and a hip torque, so K is a two-by-six matrix, and I scheduled it by leg length.

（中文要点：离线 Riccati，在线 u=−K s；六状态两输入，K 是 2×6，按腿长调度。）

追问方向：What do Q and R mean? → Q 惩罚状态误差、R 惩罚控制量；只有相对大小重要；用 Bryson 规则取初值。

**Q2. Why did you schedule the gains by leg length?**

> Because the plant changes with the leg length. The mass matrix and the gravity term in the linearized model depend on it, so A and B change: in my model the unstable pole scales roughly like one over the square root of the length and the input gain changes several times over the range. A fixed gain from one length degrades the convergence at other lengths and can even become unstable over a wide range. So I computed LQR gains on a grid of leg lengths and fitted each element of K with a polynomial, which I evaluate online at the current leg length.

**Q3. How did you build K(L) concretely?**

> Offline, I build A and B at a set of leg lengths covering the working range, choose Q and R, solve the Riccati equation, and get a two-by-six K at each length. Then I fit each of the twelve elements with a low-order polynomial in the leg length, check the fit error and check closed-loop stability on a dense grid, and store the coefficients. Online, the controller clamps the leg length to the fitted range and evaluates the polynomials. I didn't solve Riccati on the microcontroller.

（本项目取值：三次多项式、48 个系数、$Q_\phi$ 最大；记不清具体小数时说 "low-order polynomial" 即可。）

**Q4. How did you choose Q and R, and how did you tune on the real robot?**

> I started from Bryson's rule: one over the square of the acceptable error for each state and for each input. The body pitch got the largest weight. Then on the robot I looked at torque saturation and oscillation: a larger pitch weight makes the body stiffer but raises the hip-torque demand roughly in proportion, so I balanced it against the motor limit. I also ramped the velocity command so that a step does not demand a huge wheel torque.

**Q5. LQR is a linear design. What happens far from the equilibrium or when the torque saturates?**

> The linear model is valid for small angles. When the torque saturates, the effective gain drops and the loop can lose its margin, so I keep the commands smooth, limit the leg length rate, and treat large disturbances with separate recovery logic. LQR also doesn't know about contact, so I switch modes when the wheel leaves the ground.

---

## 15. 本章要点速记

1. LQR = 离线设计（模型 + $Q,R$ → Riccati → $K$）+ 在线执行（$u=-K\mathbf s$）。
2. 代价 $J=\int(\mathbf s^\top Q\mathbf s+\mathbf u^\top R\mathbf u)dt$；只有 $Q/R$ 的相对大小重要；Bryson：权重 = 1/(容许值)²。
3. ARE：$A^\top P+PA-PBR^{-1}B^\top P+Q=0$；$K=R^{-1}B^\top P$；$V=\mathbf s^\top P\mathbf s$ 是李雅普诺夫函数。
4. 标量例：极点 $-\sqrt{a^2+b^2q/r}$；双积分器 $Q=I,R=1$：$K=[1,\sqrt3]$。
5. 六状态双输出：$K$ 是 $2\times6$；第 1 行出 $T$，第 2 行出 $T_p$；非最小相位导致速度项符号看似"反直觉"。
6. $k_{25}\approx130$ N·m/rad ⇒ 俯仰每 1° 约 2.3 N·m 髋力矩，决定线性区与限幅关系。
7. 固定增益：0.10–0.30 m 内性能退化约 40%，0.4 m 以上失稳（简化模型）。
8. 调度流程：腿长网格 → 逐点 ARE → 12 个元素逐个拟合（三次，共 48 系数）→ 区间内验证 → 在线 clamp + Horner。
9. 速度指令要斜坡；腿长要限速；$L_0$ 要限幅；输出要限幅。
10. 课程用腿高 $y$ 调度四状态单输出；P7 用虚拟腿长调度六状态双输出，**以 CV 为准**。

## 自测题

1. 写出 LQR 的代价函数，说明 $Q$、$R$ 的作用。（答案见第 2 节。）
2. 手算标量系统 $a=1,b=1,q=1,r=1$ 的 $K$ 和闭环极点。（答：$K=2.414$，极点 $-1.414$。）
3. 为什么多输入 LQR 的 $K$ 是 $2\times6$？每一行的含义？（答：每个输入一行，每行六个状态增益。）
4. 为什么速度偏快时 $T$ 反而为正？（答：非最小相位，需先加速轮子使身体后倾。）
5. 为什么要把 $L_0$ 限幅在拟合区间？（答：多项式外推误差大，$L_0$ 异常值会产生荒谬增益。）

## 相关链接

- 上一章：[04_动力学建模与状态方程.md](./04_动力学建模与状态方程.md)
- 下一章：[06_VMC原理与关节力矩映射.md](./06_VMC原理与关节力矩映射.md)
- 基础：[LQR 最优控制](../../02_基础知识/04_控制理论/04_LQR最优控制.md)、[最优化与二次规划](../../02_基础知识/01_数学/04_最优化与二次规划.md)、[数字控制与实时性](../../02_基础知识/04_控制理论/08_数字控制与实时性.md)
- 八股：[控制八股](../../03_八股知识/03_机器人学/02_控制八股.md)
- 数值脚本：[13_手算与数值验证附录.md](./13_手算与数值验证附录.md)
- 相关项目：[P1 球形机器人平衡控制](../P1_AIRS球形机器人平衡控制/README.md)（四状态 LQR 作模型参考）
- 笔记原文：[P7 逐字稿](../../05_学习笔记归档/P7/P7_本对话问答逐字稿.md)（第 17–21 组）
