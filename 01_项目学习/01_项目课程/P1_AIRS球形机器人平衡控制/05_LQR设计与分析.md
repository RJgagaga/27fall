# 05 LQR 设计与分析

> **一句话定位**：LQR 在 P1 里的角色是**分析工具**，不是上机运行的控制器：它用第 04 章的四状态模型回答"位置、速度、倾角、倾角速度这四个量，应该以什么比例一起参与反馈""稳定性和控制力度之间怎么折中"，从而为级联 PID 的整定、状态异常和执行器饱和的定位提供**模型化参照**（CV 与导师推荐信的原话）。

**本章学完你能：**

- 说清 $Q$、$R_u$、$P$、$K$ 各自是什么、谁来选、谁来算，以及"为什么控制律是 $-K\mathbf z$"；
- 手算一个标量 LQR 和一个双积分器 LQR，看到"权重比 → 增益 → 极点 → 控制力度"的链条；
- 对第 04 章的四状态模型做出 LQR 设计，读懂权重扫描表；
- 把**级联 PID 展开成等效全状态反馈**，用 LQR 增益做"参照"，并用三种方法（解释增益、定位状态异常、定位饱和）实际使用它；
- 说明为什么 LQR 不是直接给 PID 抄参数，以及这个分析的边界；
- 回答 LQR 相关的英文面试题（含 "why not deploy LQR?"）。

**前置知识：**

- 第 04 章四状态模型：[04_倒立摆建模与线性化.md](./04_倒立摆建模与线性化.md)
- LQR 基础：[../../02_基础知识/04_控制理论/04_LQR最优控制.md](../../02_基础知识/04_控制理论/04_LQR最优控制.md)
- 状态空间与稳定性：[../../02_基础知识/04_控制理论/03_状态空间_可控可观_稳定性.md](../../02_基础知识/04_控制理论/03_状态空间_可控可观_稳定性.md)
- 最优化基础：[../../02_基础知识/01_数学/04_最优化与二次规划.md](../../02_基础知识/01_数学/04_最优化与二次规划.md)
- 数值离散化：[../../02_基础知识/01_数学/05_数值方法与离散化.md](../../02_基础知识/01_数学/05_数值方法与离散化.md)

**目录**

1. LQR 要解决什么问题
2. 符号、代价函数与 Riccati 方程
3. 手算 1：标量系统
4. 手算 2：双积分器
5. 四状态模型的 LQR（教学数值）
6. 权重扫描：稳定性与控制力度的折中
7. 离散化、采样与延迟
8. 把 LQR 当作 PID 整定的参照
9. 三种用法：解释增益 / 定位状态异常 / 定位饱和
10. 局限与诚实边界
11. 面试问答 / 速记 / 自测 / 链接

---

## 1. LQR 要解决什么问题

**问题：** 已知线性模型 $\dot{\mathbf z}=A\mathbf z+B u$，想要一个反馈 $u=-K\mathbf z$ 使系统稳定，并且**让"状态偏差"和"控制用力"的总代价最小**。

- 用力小 → 省电、不容易撞执行器上限、对噪声不敏感；
- 偏差小 → 稳、准、快。

两者互相矛盾（要快就要用力）。LQR 的本质是把这个矛盾**写成一个数**（代价），然后精确求解。名字：**L**inear（线性模型）、**Q**uadratic（二次代价）、**R**egulator（调节到零）。

**为什么 P1 里它是"分析"而不是"部署"：** 部署用的是级联 PID（容易在实机上整定、容易加限幅/抗积分饱和/开关）；LQR 在这里提供**一把量尺**：给定简化模型，"理想的反馈长什么样"。导师信里的原话是：该分析成为"解释已部署控制器增益、定位状态异常和执行器饱和的参考"。

---

## 2. 符号、代价函数与 Riccati 方程

| 符号 | 含义 | 谁决定 |
|---|---|---|
| $\mathbf z\in\mathbb R^n$ | 状态（相对平衡点的偏差）。本项目 $[x,\dot x,\theta,\dot\theta]^{\mathsf T}$ | 建模 |
| $u$ | 控制输入（模型里是力矩 $\tau$） | 建模 |
| $A,B$ | 线性模型 | 物理参数（第 04 章） |
| $Q\succeq0$ | 状态权重矩阵（常取对角） | **设计者** |
| $R_u>0$ | 输入权重（避免与球半径 $R$ 混淆，记作 $R_u$） | **设计者** |
| $P\succeq0$ | Riccati 方程的解，$\mathbf z^{\mathsf T}P\mathbf z$ = "从 $\mathbf z$ 出发的最小总代价" | **求解器** |
| $K$ | 反馈增益矩阵，$1\times4$ | **由 $P$ 算出** |

**代价函数**

$$J=\int_0^\infty\Big(\mathbf z^{\mathsf T}Q\mathbf z+R_u\,u^2\Big)\,\mathrm dt$$

对角 $Q=\operatorname{diag}(q_x,q_v,q_\theta,q_\omega)$ 时，$\mathbf z^{\mathsf T}Q\mathbf z=q_xx^2+q_v\dot x^2+q_\theta\theta^2+q_\omega\dot\theta^2$：每个状态偏差的平方乘一个"扣分系数"。

**解（直接给结论，推导见基础文档与下面的一段）：**

$$A^{\mathsf T}P+PA-P B R_u^{-1}B^{\mathsf T}P+Q=0\quad(\text{连续代数 Riccati 方程，CARE})$$
$$K=R_u^{-1}B^{\mathsf T}P,\qquad u=-K\mathbf z$$

**一段推导的思路（理解"为什么长这样"）：** 设"最小总代价"是 $V(\mathbf z)=\mathbf z^{\mathsf T}P\mathbf z$。在任一时刻，最优的 $u$ 要使"当下花费 + 状态变化引起的未来代价变化"最小：

$$\min_u\Big[\mathbf z^{\mathsf T}Q\mathbf z+R_uu^2+2\mathbf z^{\mathsf T}P(A\mathbf z+Bu)\Big]$$

对 $u$ 求导置零：$2R_uu+2B^{\mathsf T}P\mathbf z=0\Rightarrow u=-R_u^{-1}B^{\mathsf T}P\mathbf z$。代回，要求对任意 $\mathbf z$ 成立的方程就是 Riccati 方程。

**符号 $-K$ 的来源：** 取负号 = 负反馈（偏差为正，输出就向负方向"拉回"）。不过 $K$ 的各元素不保证都为正——真实方向还取决于 $A,B$ 的符号（例如本项目位置增益的符号就与"直觉"相反，见下）。

**LQR 的三个好性质：** ① 只要 $(A,B)$ 可控，$Q,R_u$ 取合适，**闭环一定稳定**；② 对增益有不错的裕度（连续单输入情形：增益可减半、可无限增大而保持稳定，理想模型下）；③ 求解只涉及矩阵运算，一次性离线完成。

---

## 3. 手算 1：标量系统

系统 $\dot x=a\,x+b\,u$，取 $a=1$（不稳定，$e^t$ 发散），$b=1$。代价 $J=\int(qx^2+ru^2)\,\mathrm dt$。

标量 Riccati：$2ap-\dfrac{b^2}{r}p^2+q=0$。解正根：

$$p=\frac{r}{b^2}\Big(a+\sqrt{a^2+\frac{b^2q}{r}}\Big),\qquad K=\frac{bp}{r}=\frac{a+\sqrt{a^2+b^2q/r}}{b}$$

闭环：$\dot x=(a-bK)x$。代入 $a=b=1$：$K=1+\sqrt{1+q/r}$，闭环极点 $=-\sqrt{1+q/r}$。

| $q/r$ | $K$ | 闭环极点 | $x_0=1$ 时初始控制量 $u(0)=-K$ | 衰减时间常数 $1/(K-1)$ |
|---|---|---|---|---|
| 0（完全不在乎偏差） | 2 | $-1$ | $-2$ | 1.0 |
| 3 | 3 | $-2$ | $-3$ | 0.5 |
| 8 | 4 | $-3$ | $-4$ | 0.33 |
| 24 | 6 | $-5$ | $-6$ | 0.2 |

**读这张表：**

1. 即使 $q=0$，LQR 也会把不稳定极点 $+1$ **镜像**到 $-1$——"最省力的稳定化"；
2. $q/r$ 越大，闭环越快（极点更靠左）——**稳定性/快速性变好**；
3. 同时 $K$ 越大，起始控制量越大，对测量噪声越敏感——**控制力度变大**；
4. 这就是**"稳定性 vs 控制量"折中**的最小实例，也是 CV 那句 "analyse the stability–control-effort trade-off of state weights" 的数学内核。

---

## 4. 手算 2：双积分器

系统：位置 $y$，速度 $v$，$\ddot y=u$。$A=\begin{bmatrix}0&1\\0&0\end{bmatrix}$，$B=\begin{bmatrix}0\\1\end{bmatrix}$，$Q=\operatorname{diag}(q,0)$，$R_u=r$。

设 $P=\begin{bmatrix}p_1&p_2\\p_2&p_3\end{bmatrix}$，Riccati 方程逐项：

- (1,1) 项：$-p_2^2/r+q=0\Rightarrow p_2=\sqrt{qr}$；
- (2,2) 项：$2p_2-p_3^2/r=0\Rightarrow p_3=\sqrt{2p_2r}$；
- (1,2) 项：$p_1-p_2p_3/r=0$（不需要，$K$ 只用 $p_2,p_3$）。

$K=R_u^{-1}B^{\mathsf T}P=\big[\,p_2/r,\ p_3/r\,\big]$。取 $q=1,r=1$：$p_2=1,\ p_3=\sqrt2$，

$$K=[\,1,\ \sqrt2\,],\qquad\text{闭环特征方程 }s^2+\sqrt2\,s+1=0$$

极点 $-0.707\pm0.707\mathrm j$（阻尼比 0.707）。再取 $q=4,r=1$：$K=[2,2]$，极点 $-1\pm\mathrm j$。

**结论（直接对应 PID）：** 位置权重 $q$ 只在位置项里，却自动给出了"位置比例 + 速度阻尼"的 **PD 结构**，且 $k_{\text{pos}}=\sqrt{q/r}$。位置越被重视，增益越高、越快。这解释了为什么"位置外环必须带速度项"（阻尼）。

---

## 5. 四状态模型的 LQR（教学数值）

使用第 04 章教学模型：$m=1,J_b=0,R=1,M=1,l=1,I=1,g=10$（含反作用项）：

$$A=\begin{bmatrix}0&1&0&0\\0&0&-3.333&0\\0&0&0&1\\0&0&6.667&0\end{bmatrix},\quad B=\begin{bmatrix}0\\1\\0\\-1\end{bmatrix}$$

### 5.1 怎么选 $Q,R_u$（常用起步法：Bryson 规则）

为每个量设定"可以接受的最大偏差"，权重取其平方的倒数：

$$Q_{ii}=\frac{1}{(z_{i,\max})^2},\qquad R_u=\frac{1}{u_{\max}^2}$$

**教学示例：** $|x|\le0.5$ m，$|\dot x|\le0.5$ m/s，$|\theta|\le0.1$ rad，$|\dot\theta|\le1$ rad/s，$|\tau|\le2$ N·m：

$$Q=\operatorname{diag}(4,\ 4,\ 100,\ 1),\qquad R_u=0.25$$

这只是**起点**，不是答案。倾角权重最大，因为倾角容差最小。

### 5.2 求解结果

用 `solve_continuous_are`（第 11 章给代码）得到（教学数值）：

$$K=[-4.00,\ -10.52,\ -46.11,\ -20.73]$$

控制律 $\tau=-K\mathbf z=4.00\,x+10.52\,\dot x+46.11\,\theta+20.73\,\dot\theta$。

**符号怎么读：**

- 倾角与倾角速度项系数为正：机体前倾（$\theta>0$）或在加速前倾时，给**正**力矩，让球向前滚去"接住"——合理；
- 位置、速度项系数也是正：**位置在前方（$x>0$）反而给正力矩，让球进一步向前！**这与"偏差为正就往回拉"的直觉相反——因为非最小相位：要想把机器人拉回后方，必须先把机体倾向后方（第 04 章），而要让机体后倾，球要先**向前**冲。所以正确的位置反馈在力矩层是"反号"的，经倾角通道才表现为"往回走"。

闭环极点：$-4.47\pm1.88\mathrm j$（快，倾角/平衡模态）、$-0.64\pm0.40\mathrm j$（慢，位置/速度模态）。两组频率相差约 7 倍——天然的"内快外慢"结构。

### 5.3 仿真：初始倾角 0.05 rad 恢复（教学数值）

线性模型 + 同一控制器，$\mathbf z_0=[0,0,0.05,0]$：

- 峰值力矩 $|\tau|_{\max}=2.31$ N·m（$=K_{p\theta}\times0.05$，发生在 $t=0$；略超设计时设的 2 N·m 上限，说明 Bryson 起步值需要再修）；
- 球的最大位移约 9.3 cm；
- 约 4.2 s 后进入 $|\theta|<0.005$ rad 且 $|x|<0.02$ m。

位置模态慢（极点 $-0.64$），这是调节"位置回得慢"的瓶颈。

### 5.4 位置阶跃：非最小相位的实感

把目标位置设为 $x_t=1$ m（级联形式，见 §8；姿态参考限幅 $\pm0.1$ rad），教学数值仿真：

| 时刻 (s) | 球位置 $x$ (m) | 倾角 $\theta$ (rad) | 力矩 $\tau$ (N·m) |
|---|---|---|---|
| 0.1 | −0.013 | 0.013 | −0.11 |
| 0.5 | **−0.061** | 0.072 | 0.70 |
| 1.0 | 0.003 | 0.072 | 0.63 |
| 2.0 | 0.347 | −0.003 | −0.08 |
| 4.0 | 0.838 | −0.025 | −0.17 |
| 8.0 | 1.008 | −0.002 | −0.01 |

观察：

1. $t=0$ 时力矩 $=-4.0$（负）：球**先向后**（最小 −6.2 cm @0.56 s），机体前倾到 0.07 rad；
2. 然后球向前，倾角逐渐回到零；末段倾角略为负，用来**刹车**；
3. 最后约 5.1 s 到达 95%，最大倾角 0.084 rad。

**姿态参考限幅的影响：**

| 参考限幅 (rad) | 最大倾角 (rad) | 最大 $|\tau|$ (N·m) | 到 95% 位置时间 (s) | 反向 undershoot (m) |
|---|---|---|---|---|
| 0.02 | 0.023 | 0.92 | 6.9 | −0.013 |
| 0.05 | 0.055 | 2.31 | 5.6 | −0.031 |
| 0.10 | 0.084 | 4.00 | 5.1 | −0.062 |
| 0.20 | 0.094 | 4.00 | 5.0 | −0.079 |

限幅越小：越温和、越慢、undershoot 越小；限幅从 0.1 放宽到 0.2 以上，变化已很小（此时力矩峰值 4.0 来自 $t=0$ 的阶跃跳变——目标一下子跳到 1 m，位置项瞬间要求 −4.0 N·m）。**这就是"有界姿态参考"的量化含义**，第 06 章展开。

---

## 6. 权重扫描：稳定性与控制力度的折中

同一模型、同一初值（$\theta_0=0.05$ rad），改变权重（教学数值，线性仿真）：

| 方案 | $K=[k_x,k_v,k_\theta,k_\omega]$ | 闭环极点（主要的） | 峰值 $|\tau|$ | 最大位移 | 稳定时间 |
|---|---|---|---|---|---|
| 基准 $Q=(4,4,100,1)$，$R_u=0.25$ | −4.0, −10.5, −46.1, −20.7 | −4.5±1.9j; −0.64±0.40j | 2.31 | 9.3 cm | 4.2 s |
| $q_\theta\times10$ | −4.0, −14.2, −84.3, −27.7 | −6.4±4.9j; −0.35±0.29j | 4.21 | 7.7 cm | 约 6.1 s（位置模态变慢） |
| $R_u\times10$（更省力） | −1.3, −3.9, −24.3, −10.8 | −2.9±1.0j; −0.54±0.39j | 1.21 | 12.6 cm | 5.6 s |
| $R_u/10$（更激进） | −12.7, −31.5, −116, −51.7 | −13.6; −5.3; −0.65±0.40j | 5.82 | 7.7 cm | 3.7 s |
| $q_x\times10$（更重视位置） | −12.7, −21.8, −67.0, −33.1 | −4.6±2.2j; −1.1±0.6j | 3.35 | 8.9 cm | 2.5 s |

**读法与规律：**

1. $R_u$ 越大 → $K$ 越小 → 峰值力矩小、响应慢、位移大：**更省力，更松弛**；$R_u$ 越小则相反。
2. $q_\theta$ 增大 → $k_\theta,k_\omega$ 明显变大，峰值力矩上升——**重视倾角 = 更大的瞬时力矩需求**，更容易撞上执行器上限；
3. $q_x$ 增大 → 位置增益上升，位置模态变快（2.2 s），代价是倾角增益、力矩也上升；
4. 观察到 $|k_x|=\sqrt{q_x/R_u}$（所有方案精确成立，例如：$\sqrt{4/0.25}=4$，$\sqrt{40/0.25}=12.65$，$\sqrt{4/0.025}=12.65$）。原因是位置 $x$ 不出现在动力学里（$A$ 的第一列全 0），这和 §4 的双积分器同理。
5. 不存在"免费午餐"：提高稳定速度必然意味着更大 $K$、更大力矩、更强的噪声放大（$\tau_{\text{噪声}}\approx k_\omega\sigma_{\text{gyro}}$，例如 $\sigma_{\text{gyro}}=0.01$ rad/s、$k_\omega=20.7$ → 0.21 N·m 的力矩抖动）。

> **提示：** 这里所有数字都是"教学数值"，用来演示**规律**。把它们说成你实机的结果，是不允许的。

---

## 7. 离散化、采样与延迟

### 7.1 连续 vs 离散 LQR

控制器是数字的（周期 $T_s$）。两种做法：

- 连续设计后直接用：$K_c$ 设计时把系统当连续的，上机时每 $T_s$ 更新一次（零阶保持）。$T_s$ 足够小，几乎一样；
- 先离散化再解**离散 Riccati**（`solve_discrete_are`）：$K_d$。

教学数值，同一 $Q,R_u$（代价按 $T_s$ 积分，$Q_d=QT_s$，$R_d=R_uT_s$）：

| $T_s$ | $K_d=[k_x,k_v,k_\theta,k_\omega]$（$\tau=-K_d\mathbf z_k$） |
|---|---|
| 连续 $K_c$ | −4.00, −10.52, −46.11, −20.73 |
| 1 ms | −3.98, −10.47, −45.93, −20.65 |
| 10 ms (100 Hz) | −3.80, −10.02, −44.32, −19.92 |
| 20 ms | −3.61, −9.54, −42.61, −19.15 |
| 50 ms | −3.10, −8.24, −37.98, −17.06 |

100 Hz 时与连续设计相差约 5%，结论一致；周期越长偏差越大。

### 7.2 延迟与采样对稳定性的伤害（教学模型）

把连续设计的 $K_c$ 用到系统上，加纯延迟（线性仿真）：

- 延迟 ≤ 110 ms 仍稳定，**≥ 120 ms 失稳**；
- 采样保持周期 $T_s$ 至 100 ms 仍稳定（此时延迟余量已被吃光）。

这个教学模型的不稳定极点是 2.58 s⁻¹。若是第 04 章"量级示意"那台（极点 5.87 s⁻¹），用较激进的权重（$R_u=0.01$）做同样思路的仿真，延迟余量只有约 **20–30 ms**，即：极点越快，对延迟越敏感。这给出了一个有用的规则：**总延迟（传感 + 滤波 + 通信 + 电机）应远小于 $1/\lambda$**。

### 7.3 效率问题（面试常问）

- 离线：解 Riccati 是 $n\times n$ 矩阵运算（$n=4$），毫秒级，只做一次；
- 在线：$u=-K\mathbf z$ 是 4 次乘加；比 PID 还简单；
- 所以"效率"从来不是 LQR 不被上机的理由；真正的原因是模型简化、整定与可调性、限幅/积分/开关处理、与已有 PID 架构的一致性（见 §10）。

---

## 8. 把 LQR 当作 PID 整定的参照

### 8.1 级联 PID 展开成等效全状态反馈

设内环是 PD：$\tau=K_{p\theta}(\theta-\theta_{\text{ref}})+K_{d\theta}\dot\theta$（符号按第 04 章约定：前倾 $\theta>0$ 需正力矩）；外环是 PD：

$$\theta_{\text{ref}}=K_{px}(x_t-x)+K_{dx}(v_t-\dot x)$$

代入（目标取 $x_t=v_t=0$ 看调节）：

$$\tau=K_{p\theta}\theta+K_{d\theta}\dot\theta+K_{p\theta}K_{px}\,x+K_{p\theta}K_{dx}\,\dot x$$

与 $\tau=-K\mathbf z=-[k_x,k_v,k_\theta,k_\omega]\mathbf z$ 对照，**一一对应**：

$$K_{p\theta}=-k_\theta,\quad K_{d\theta}=-k_\omega,\quad K_{px}=\frac{k_x}{k_\theta},\quad K_{dx}=\frac{k_v}{k_\theta}$$

基准 LQR 下：$K_{p\theta}=46.1$，$K_{d\theta}=20.7$，$K_{px}=0.0867$ rad/m，$K_{dx}=0.228$ rad/(m/s)。

**意义：** 级联 PID 是全状态反馈的一种**结构化**特例。有了这个对应，就能：把部署的 PID 增益换算成等效 $K$，在同一个模型上算闭环极点，并和 LQR 增益比。

> **量级检查（提示，需核对）：** 学习笔记里曾记录过某个快照中外环增益约为 0.06 rad/m、0.3 rad/(m/s)。它和上面教学模型换算出的 0.087、0.23 同量级——但教学参数不是你的机器人，这只是数量级的"健全性检查"，不是证据。你应拿**自己的真实质量/惯量**重做这一步。

### 8.2 "参照"不等于"抄参数"

- 单位不同：LQR 输出模型力矩，PID 输出可能是速度命令或缩放量，需要换算；
- 结构不同：PID 有积分、限幅、滤波、前馈；LQR 没有；
- 所以用法是：**比较比例关系与行为趋势**，不是复制数字。

---

## 9. 三种用法

### 9.1 用法一：解释已部署控制器的增益

做法：① 用实机参数建模（第 04 章）；② 把部署的 PID 展开成等效 $K_{\text{pid}}$（§8.1），注意单位换算；③ 在模型上算 $A-BK_{\text{pid}}$ 的极点；④ 与 LQR 对比。

可读出的信息（教学模型为例）：

- **灵敏度表**——把基准 LQR 的某个增益单独缩放：

| 单独改动 | 结果（最大实部，正 = 失稳） |
|---|---|
| $k_\omega$ 变 0.5 倍（阻尼不足） | **+0.57 失稳** |
| $k_\omega$ 变 2 倍 | −0.31 稳定 |
| $k_v$ 变 2 倍（速度环太硬） | **+1.07 失稳** |
| $k_\theta$ 变 0.5 / 2 倍 | −0.26 / −0.22 稳定 |
| $k_x$ 变 0.5 / 2 倍 | −0.26 / −0.58 稳定 |
| 全部 $K$ 同乘 0.2 | **+0.36 失稳**；乘 0.3 则稳定 |
| 任何一个增益**符号取反** | **全部失稳** |

- 结论：**阻尼（$k_\omega$）不足和速度环过硬最危险**；增益整体低于约 $0.25K$ 就会失稳（与"重力要被克服"对应）；符号错一定炸。
- 对应到 PID：内环 $D$ 项（陀螺）不能太小；外环速度项不能太大；这是调参方向的有力提示。

### 9.2 用法二：定位状态异常

模型给出了"正常工作时各状态之间应有的关系"，偏离即线索。

**（a）倾角零点偏置。** 假设倾角测量有常值偏置 $b$（IMU 安装误差、零点没标好）。稳态时真实 $\theta=0$，要求 $\tau=0$：

$$0=-\big(k_xx+k_\theta b\big)\ \Rightarrow\ x_{ss}=-\frac{k_\theta}{k_x}\,b$$

教学数值：$k_\theta/k_x=11.5$ m/rad。$b=1^\circ=0.01745$ rad → $x_{ss}=-0.20$ m（仿真 −0.2015 m，吻合）。**所以**：机器人静止时如果缓慢爬到某个位置才停下，且偏移量与位置增益比成正比，先查 IMU 零点和初始化偏置（第 03、08 章）。

**（b）符号/轴错误。** 任何一个增益符号取反都失稳（上表）。如果开环测试里"一使能就往一个方向加速倒下"，先怀疑 IMU 轴/符号/轮序（第 02、09 章）。

**（c）噪声通道。** 力矩噪声 $\approx k_\omega\sigma_{\text{gyro}}$。若实机在静止时轮命令高频抖动，拿陀螺噪声和 $K_{d\theta}$ 相乘估计一下，就知道是不是 D 项放大了噪声（第 06、09 章）。

### 9.3 用法三：定位执行器饱和

**问题：** 机器人倒了，是"控制没调好"还是"执行器根本不够力"？

**模型的回答：** 对给定的 $K$ 和力矩上限 $\tau_{\max}$，存在**最大可恢复倾角**。教学数值（基准 $K$，非线性仿真，初速度为零）：

| $\tau_{\max}$ (N·m) | 最大可恢复初始倾角 (rad) |
|---|---|
| 0.5 | ≈0.05 |
| 1 | ≈0.11 |
| 2 | ≈0.23 |
| 4 | ≈0.45 |

约为 $\theta_{\max}\approx0.11\,\tau_{\max}$（粗上界 $\tau_{\max}/(\text{重力项系数})=\tau_{\max}/6.67=0.15\,\tau_{\max}$）。

**怎么用：** 在实机日志里看"倾角达到多大时轮命令贴上限"：

- 如果贴限幅发生在**很小的倾角**（远小于 $\theta_{\max}$ 的模型预测），说明增益过高/噪声过大/滤波不当，不是真的需要那么大的力；
- 如果贴限幅发生在**接近** $\theta_{\max}$，说明执行器余量本身不够（电机/电池/限幅设得太保守），需要降低外环参考上限或提高限幅/电压；
- 饱和还会降低**有效增益**；降到约 $0.25K$ 以下就进入失稳区，这也是为什么饱和时倾角会"加速倒下"。

---

## 10. 局限与诚实边界

1. **模型简化**：单方向平面；忽略三轮细节、球面接触、手臂动态、摩擦、延迟。它回答的是定性/半定量问题；
2. **输入单位**：模型输入是力矩，实机输出可能是轮速命令。换算要显式做；
3. **未部署**：CV 与推荐信都是"用于分析"。你的原话应是 "I used LQR as an analysis tool to interpret and diagnose the deployed PID"，不是 "I deployed LQR"；
4. **LQR 保证只在线性区**：大倾角、饱和下没有稳定保证；
5. **笔记里的脚本提醒**：学习笔记记录的脚本用连续 LQR 算 $K$，之后又手动缩放了某些分量并换算成 PID 比例关系，写出 `model_params.yaml`。缩放后的 $K$ 已不是原 $Q,R_u$ 的最优解，应称为"LQR 参照 + 人工调整"。该脚本不在当前源码目录，细节以你本人记忆为准（见 README 口径备忘）。

---

## 11. 面试问答

**Q1. What did you use LQR for in this project?**

> I used it as an analysis tool, not as the deployed controller. After the cascaded PID worked, I derived a four-state linearised model, with position, velocity, tilt and tilt rate, and used LQR to see how the state weights trade stability against control effort. That gave my team a reference for interpreting the deployed gains, for locating state anomalies, and for diagnosing actuator saturation.

**Q2（追问）. How exactly did it help you tune the PID?**

> The cascade is a structured special case of full state feedback, so I could expand the PID gains into an equivalent four-gain feedback law and compare the ratios and the closed-loop poles with the LQR solution. For example, the model showed that too little damping on the tilt rate or too stiff a velocity term is the dangerous direction, so I knew to look at the gyro term and the outer-loop velocity term first, instead of simply raising the tilt proportional gain.

**Q3. What do Q and R mean, and how did you choose them?**

> Q penalises state deviations and R penalises control effort. I started from Bryson's rule: one over the square of the largest acceptable deviation of each state, and one over the square of the largest acceptable torque. Then I swept the weights to see how the gains, the peak torque and the settling time changed. A larger R gives gentler, slower control; a larger tilt weight gives higher tilt gains and a higher peak torque.

**Q4（追问）. Why didn't you deploy LQR on the robot?**

> The model is a single-plane approximation: it ignores the three-wheel geometry, slip, arm dynamics and actuator limits, and its input is a torque while the robot interface takes wheel commands. The PID structure was easier to tune on the real robot, and to add limits, anti-windup and a feedforward. So I treated LQR as a reference that tells me what a good trade-off looks like, and the PID as the implementation.

**Q5. How can the model help you find an actuator-saturation problem?**

> For a given torque limit, the linear-plus-saturation model has a largest tilt it can recover from. If the real wheel command hits its limit at a much smaller tilt, the problem is high gain or noise rather than a real lack of authority. If it hits the limit near the predicted tilt, the actuator margin itself is the problem.

---

## 本章要点速记

1. LQR：$J=\int(\mathbf z^{\mathsf T}Q\mathbf z+R_uu^2)\mathrm dt$ → CARE → $K=R_u^{-1}B^{\mathsf T}P$ → $u=-K\mathbf z$。
2. $Q,R_u$ 是设计者的偏好；$P,K$ 由求解器算；起步用 Bryson 规则。
3. 标量例：$q/r\uparrow\Rightarrow K\uparrow$，极点更快，控制量更大：稳定性与控制力度的折中。
4. 双积分器例：位置权重自动给出 PD 结构，$k_{\text{pos}}=\sqrt{q/r}$。
5. 四状态：内快外慢两组极点；位置/速度项在力矩层符号"反直觉"（非最小相位）。
6. 级联 PID = 结构化全状态反馈，可展开、可对照，但不能直接抄数。
7. 三种用法：解释增益（灵敏度表）、定位状态异常（$x_{ss}=-\frac{k_\theta}{k_x}b$）、定位饱和（最大可恢复倾角 ≈ $0.11\tau_{\max}$，教学数值）。
8. 延迟余量约 $1/\lambda$ 量级；离散化与 100 Hz 差别 ~5%。
9. LQR 在 P1 是分析工具，未上机；说法必须如实。

## 自测题

1. 标量系统 $a=1,b=1$，$q/r=8$，求 $K$ 与闭环极点。（$K=4$，极点 $-3$。）
2. 双积分器 $q=4,r=1$ 的 $K$？（$[2,2]$。）
3. $k_\theta/k_x=11.5$ m/rad，倾角偏置 $0.5^\circ$，稳态位置偏移？（$11.5\times0.00873=0.10$ m。）
4. $\sigma_{\text{gyro}}=0.02$ rad/s，$k_\omega=20$，力矩噪声？（0.4 N·m。）
5. 为什么位置增益的力矩符号"反直觉"？（非最小相位：要先让机体后倾，球要先前冲。）

## 相关链接

- 上一章：[04_倒立摆建模与线性化.md](./04_倒立摆建模与线性化.md)　下一章：[06_级联PID控制.md](./06_级联PID控制.md)
- 复现代码与实验：[11_教学仿真实验手册.md](./11_教学仿真实验手册.md)
- 八股：[../../03_八股知识/03_机器人学/02_控制八股.md](../../03_八股知识/03_机器人学/02_控制八股.md)
- 工程化实践（频率、周期、优化与工程问题）：[20_工程化实践_频率周期与问题解决.md](./20_工程化实践_频率周期与问题解决.md)
