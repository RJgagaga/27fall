# 卡尔曼滤波与扩展卡尔曼滤波（EKF）

> **一句话定位**：卡尔曼滤波是"带不确定度的预测—修正循环"：先用模型预测，再用测量修正，修正多少由双方的不确定度（协方差）决定。EKF 是它在非线性系统上的局部线性化版本，也是 P5 定位里 "EKF/AMCL" 的 EKF 那一半。

## 本章学完你能……

- 用贝叶斯滤波的语言解释"预测"和"更新"，知道卡尔曼滤波是高斯 + 线性假设下的精确解。
- 从零推导卡尔曼滤波的五个公式：$\hat x^-=F\hat x$、$P^-=FPF^\top+Q$、$K=P^-H^\top S^{-1}$、$\hat x=\hat x^-+K\nu$、$P=(I-KH)P^-$，并解释每一项的直觉。
- 完整手算一维（位置）和二维（位置 + 速度）卡尔曼滤波，看到"只测位置，速度也被修正"的相关性效应。
- 推导 EKF：为什么要线性化、雅可比 $F$、$H$ 怎么来，并手算一次差速 / 独轮车模型的 EKF 预测 + 更新。
- 说出 $Q$、$R$ 的含义与调参方法，用 NEES / NIS 做一致性检验，用马氏距离做异常值门控。
- 用英文回答 "Explain the Kalman filter / EKF / how do you tune Q and R / why does the filter diverge" 等问题。

## 前置知识

- 高斯分布、协方差、条件概率：[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)
- 矩阵运算与求逆：[线性代数](../01_数学基础/01_线性代数.md)
- 状态空间与离散化：[状态空间、可控可观、稳定性](../05_控制理论/03_状态空间_可控可观_稳定性.md)、[数值方法与离散化](../01_数学基础/05_数值方法与离散化.md)
- 传感器误差：[传感器章](./01_传感器_IMU_编码器_激光雷达.md)

## 目录

1. 滤波问题与贝叶斯滤波
2. 线性卡尔曼滤波：模型与符号
3. 预测步推导
4. 更新步推导（五公式的来历）
5. 增益的直觉
6. 手算一：一维卡尔曼滤波
7. 手算二：二维（位置 + 速度）
8. EKF：线性化与雅可比
9. 手算三：EKF 预测 + 更新
10. Q、R 调参与一致性检验
11. 在本项目里怎么用
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记、自测题、相关链接

---

## 1. 滤波问题与贝叶斯滤波

### 1.1 它要解决什么问题

我们想知道机器人的**状态** $x_k$（位置、速度、航向……），但只能拿到两类带噪声的信息：

1. **运动信息**：控制输入或里程计，告诉我们"状态大概怎么变"（模型预测）。
2. **测量信息**：传感器读数 $z_k$，告诉我们"状态大概是多少"（观测）。

两者都不完全可信。**滤波**就是：给定到当前为止所有信息，求状态的最佳估计及其不确定度。

### 1.2 贝叶斯滤波的两步

用概率语言：目标是后验 $p(x_k\mid z_{1:k})$。递推两步：

**预测（Predict）**：用运动模型把上一时刻的分布推到下一时刻：

$$
p(x_k\mid z_{1:k-1})=\int p(x_k\mid x_{k-1})\,p(x_{k-1}\mid z_{1:k-1})\,dx_{k-1}
$$

**更新（Update）**：用新测量 $z_k$ 和贝叶斯公式修正：

$$
p(x_k\mid z_{1:k})=\eta\ p(z_k\mid x_k)\ p(x_k\mid z_{1:k-1})
$$

$\eta$ 是归一化常数。"先验 × 似然 = 后验"。

这个递推的假设是**马尔可夫性**：当前状态只依赖上一时刻状态；测量只依赖当前状态。

### 1.3 卡尔曼滤波是它的特例

积分和乘积在一般分布下无法闭式计算。但在以下条件下可以：

1. 状态转移和观测都是**线性**的；
2. 过程噪声、观测噪声都是**高斯**、零均值、彼此独立；
3. 初始分布也是高斯。

此时所有分布都保持高斯，只需要传递**均值 $\hat x$ 和协方差 $P$**。这就是卡尔曼滤波（Kalman filter）。粒子滤波（见[粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)）是在非高斯 / 多峰情况下用样本去近似这两步。

```text
        ┌────────── 预测：用模型把 (x̂, P) 向前推 ──────────┐
        │                                                   │
 (x̂_{k-1}, P_{k-1}) ──▶  (x̂_k^-, P_k^-)  ──▶ 更新：用 z_k 修正 ──▶ (x̂_k, P_k)
        ▲                                                           │
        └───────────────────────────────────────────────────────────┘
```

---

## 2. 线性卡尔曼滤波：模型与符号

### 2.1 模型

$$
x_k=F\,x_{k-1}+B\,u_k+w_k,\qquad w_k\sim\mathcal N(0,Q)
$$
$$
z_k=H\,x_k+v_k,\qquad v_k\sim\mathcal N(0,R)
$$

### 2.2 符号表

| 符号 | 含义 | 维度 |
|---|---|---|
| $x_k$ | 状态真值 | $n\times1$ |
| $\hat x_k^-$ | 先验估计（预测后、更新前） | $n\times1$ |
| $\hat x_k$ | 后验估计（更新后） | $n\times1$ |
| $P_k^-,P_k$ | 先验 / 后验协方差：$P=E[(x-\hat x)(x-\hat x)^\top]$ | $n\times n$ |
| $F$ | 状态转移矩阵 | $n\times n$ |
| $B,u$ | 控制输入矩阵与输入 | |
| $H$ | 观测矩阵：状态到测量的映射 | $m\times n$ |
| $z$ | 测量 | $m\times1$ |
| $Q$ | 过程噪声协方差（模型有多不准） | $n\times n$ |
| $R$ | 观测噪声协方差（传感器有多不准） | $m\times m$ |
| $\nu=z-H\hat x^-$ | 新息 / 残差（innovation） | $m\times1$ |
| $S=HP^-H^\top+R$ | 新息协方差 | $m\times m$ |
| $K$ | 卡尔曼增益 | $n\times m$ |

> **提示**：LQR 里也有字母 $Q,R$（代价权重），与滤波里的噪声协方差**完全不同**。两者在数学上有"对偶"关系，但含义不能混。

---

## 3. 预测步推导

**均值**：对状态方程取期望，噪声均值为零：

$$
\hat x_k^-=E[x_k]=F\hat x_{k-1}+Bu_k
$$

**协方差**：误差 $e_k^-=x_k-\hat x_k^-=F(x_{k-1}-\hat x_{k-1})+w_k=Fe_{k-1}+w_k$。因为 $e_{k-1}$ 与 $w_k$ 独立：

$$
P_k^-=E[e_k^-e_k^{-\top}]=F\,E[e_{k-1}e_{k-1}^\top]\,F^\top+E[w_kw_k^\top]=FP_{k-1}F^\top+Q
$$

两项的含义：

- $FPF^\top$：**已有的不确定度被模型传播**。例如位置误差会通过速度进入下一步的位置。
- $Q$：**这一步新增的不确定度**（模型没覆盖的扰动，比如打滑、风）。

**结论**：只预测不更新，$P$ 只会越来越大。这正是航位推算（dead reckoning）会发散的数学表达。

---

## 4. 更新步推导（五公式的来历）

### 4.1 设计：线性修正

我们想用 $z_k$ 修正先验。最自然的形式是"先验 + 增益 × 新息"：

$$
\hat x_k=\hat x_k^-+K\,(z_k-H\hat x_k^-)=\hat x_k^-+K\nu
$$

问题变成：**怎么选 $K$ 才最好？** "最好" = 后验误差的协方差（的迹）最小。

### 4.2 后验误差协方差

后验误差 $e_k=x_k-\hat x_k$。把 $z=Hx+v$ 代入：

$$
e_k=x_k-\hat x_k^--K(Hx_k+v_k-H\hat x_k^-)=(I-KH)e_k^--Kv_k
$$

先验误差 $e^-$ 与测量噪声 $v$ 独立，所以

$$
P_k=(I-KH)P_k^-(I-KH)^\top+K R K^\top\qquad(\text{Joseph 形式})
$$

### 4.3 最小化迹，求 $K$

展开：

$$
P=P^--KHP^--P^-H^\top K^\top+K(HP^-H^\top+R)K^\top=P^--KHP^--P^-H^\top K^\top+KSK^\top
$$

对 $K$ 求 $\operatorname{tr}(P)$ 的导数（用 $\partial\operatorname{tr}(KA)/\partial K=A^\top$，$\partial\operatorname{tr}(KSK^\top)/\partial K=2KS$，$S$ 对称）：

$$
\frac{\partial\operatorname{tr}P}{\partial K}=-2P^-H^\top+2KS=0\ \Rightarrow\ \boxed{K=P^-H^\top S^{-1}=P^-H^\top(HP^-H^\top+R)^{-1}}
$$

把 $K$ 代回 Joseph 形式，$KSK^\top=P^-H^\top K^\top$，三项合并：

$$
P_k=(I-KH)P_k^-
$$

### 4.4 五公式总结

**预测**

$$
\hat x_k^-=F\hat x_{k-1}+Bu_k,\qquad P_k^-=FP_{k-1}F^\top+Q
$$

**更新**

$$
S_k=HP_k^-H^\top+R,\quad K_k=P_k^-H^\top S_k^{-1},\quad \hat x_k=\hat x_k^-+K_k(z_k-H\hat x_k^-),\quad P_k=(I-K_kH)P_k^-
$$

（"五公式"指：先验均值、先验协方差、增益、后验均值、后验协方差；新息协方差 $S$ 是增益里的中间量。）

> **工程提示**：直接用 $(I-KH)P^-$ 在数值上可能让 $P$ 失去对称正定。实现时常用 Joseph 形式，或每步做 $P\leftarrow\tfrac12(P+P^\top)$。

### 4.5 贝叶斯角度的验证（一维）

先验 $\mathcal N(\hat x^-,P^-)$，似然 $\mathcal N(z,R)$。两个高斯相乘后，**精度**（方差倒数）相加，均值按精度加权：

$$
\frac1P=\frac1{P^-}+\frac1R,\qquad \hat x=P\Big(\frac{\hat x^-}{P^-}+\frac zR\Big)
$$

把它改写：$\hat x=\hat x^-+\dfrac{P^-}{P^-+R}(z-\hat x^-)$，$P=\dfrac{P^-R}{P^-+R}=(1-K)P^-$。与卡尔曼公式（$H=1$）完全一致。**两条独立推导（最小方差线性估计、高斯乘积）得到同一结果**。

---

## 5. 增益的直觉

一维、$H=1$ 时 $K=\dfrac{P^-}{P^-+R}\in[0,1]$：

| 情形 | $K$ | 含义 |
|---|---|---|
| $R\to0$（测量很准） | $\to1$ | 几乎直接取测量 |
| $R\to\infty$（测量很差） | $\to0$ | 基本不动，信预测 |
| $P^-\to0$（预测很准） | $\to0$ | 信预测 |
| $P^-\gg R$ | $\to1$ | 信测量 |

**把增益看成"信谁多一点"的权重**：权重与"各自不确定度的倒数"成正比。

多维时 $K$ 的作用更丰富：$P^-H^\top$ 里的**非对角元**把对"能测到的状态"的修正分配给"测不到的状态"。比如只测位置，却能修正速度——前提是预测协方差 $P^-$ 里位置和速度**相关**。这在第 7 节手算里会看到。

后验协方差总比先验小（$P\preceq P^-$）：每次测量都使不确定度下降，不会上升。预测使不确定度上升，更新使其下降；二者平衡后得到稳态协方差。

---

## 6. 手算一：一维卡尔曼滤波

**设定**（教学数值）：机器人沿直线走，每步命令前进 $u=1$ m（所以 $F=1,\ B=1$）；用 GPS 式位置测量（$H=1$）。

- 初值 $\hat x_0=0$，$P_0=4\ \text{m}^2$；
- $Q=1\ \text{m}^2$（每步的运动不确定度）；$R=2\ \text{m}^2$（测量方差）；
- 测量：$z_1=1.5$，$z_2=2.6$。

### 第 1 步

**预测**：
$\hat x_1^-=0+1=1$；$P_1^-=4+1=5$。

**更新**：
$S=P^-+R=5+2=7$；$K=5/7=0.7143$；
新息 $\nu=1.5-1=0.5$；
$\hat x_1=1+0.7143\times0.5=1.3571$；
$P_1=(1-0.7143)\times5=1.4286$。

**验证（精度加权）**：$\frac1{5}+\frac12=0.7$，$P=1/0.7=1.4286$ ✓；$\hat x=1.4286\times(\frac15+\frac{1.5}2)=1.4286\times0.95=1.3571$ ✓。

### 第 2 步

**预测**：$\hat x_2^-=1.3571+1=2.3571$；$P_2^-=1.4286+1=2.4286$。

**更新**：$S=2.4286+2=4.4286$；$K=2.4286/4.4286=0.5484$；
$\nu=2.6-2.3571=0.2429$；
$\hat x_2=2.3571+0.5484\times0.2429=2.4903$；
$P_2=(1-0.5484)\times2.4286=1.0968$。

**读结果**：

| 步 | $\hat x^-$ | $P^-$ | $K$ | $\nu$ | $\hat x$ | $P$ |
|---|---|---|---|---|---|---|
| 1 | 1.0000 | 5.0000 | 0.7143 | 0.5000 | 1.3571 | 1.4286 |
| 2 | 2.3571 | 2.4286 | 0.5484 | 0.2429 | 2.4903 | 1.0968 |

增益在下降（0.71 → 0.55）：随着估计变得更确定，对新测量的依赖在降低。协方差会收敛到稳态值（预测增加 $Q$ 与更新减少相平衡的点），增益也随之稳定，这就是稳态卡尔曼滤波，计算时可以直接用常数增益。

---

## 7. 手算二：二维（位置 + 速度）

### 7.1 设定

状态 $x=[p,\ v]^\top$，匀速模型（噪声为加速度扰动）。$\Delta t=1$ s：

$$
F=\begin{bmatrix}1&1\\0&1\end{bmatrix},\quad H=\begin{bmatrix}1&0\end{bmatrix}\ (\text{只测位置}),\quad R=[1]
$$

过程噪声用"离散白噪声加速度模型"：加速度噪声谱强度 $q$，则

$$
Q=q\begin{bmatrix}\Delta t^3/3&\Delta t^2/2\\ \Delta t^2/2&\Delta t\end{bmatrix}
$$

为了让数字干净，这里用一个常见的简化形式 $Q=q\begin{bmatrix}1/4&1/2\\1/2&1\end{bmatrix}$（把加速度当作每步恒定的随机量，$\Delta t=1$），取 $q=0.2$：

$$
Q=\begin{bmatrix}0.05&0.1\\0.1&0.2\end{bmatrix}
$$

初值：$\hat x_0=[0,\ 1]^\top$，$P_0=I$。测量 $z=1.6$。

### 7.2 预测

$$
\hat x^-=F\hat x_0=\begin{bmatrix}0+1\cdot1\\1\end{bmatrix}=\begin{bmatrix}1\\1\end{bmatrix}
$$

$$
FP_0F^\top=\begin{bmatrix}1&1\\0&1\end{bmatrix}\begin{bmatrix}1&0\\0&1\end{bmatrix}\begin{bmatrix}1&0\\1&1\end{bmatrix}=\begin{bmatrix}2&1\\1&1\end{bmatrix}
$$

$$
P^-=FP_0F^\top+Q=\begin{bmatrix}2.05&1.1\\1.1&1.2\end{bmatrix}
$$

注意：虽然 $P_0$ 是对角阵（位置和速度不相关），预测后出现了 **非对角元 1.1**——因为位置由速度积分得到，速度误差进入位置误差，二者相关。

### 7.3 更新

$$
S=HP^-H^\top+R=2.05+1=3.05
$$

$$
K=P^-H^\top S^{-1}=\frac1{3.05}\begin{bmatrix}2.05\\1.1\end{bmatrix}=\begin{bmatrix}0.6721\\0.3607\end{bmatrix}
$$

新息 $\nu=z-H\hat x^-=1.6-1=0.6$。

$$
\hat x=\hat x^-+K\nu=\begin{bmatrix}1+0.6721\times0.6\\1+0.3607\times0.6\end{bmatrix}=\begin{bmatrix}1.4033\\1.2164\end{bmatrix}
$$

协方差：$P=(I-KH)P^-$，其中

$$
I-KH=\begin{bmatrix}1-0.6721&0\\-0.3607&1\end{bmatrix}=\begin{bmatrix}0.3279&0\\-0.3607&1\end{bmatrix}
$$

$$
P=\begin{bmatrix}0.3279\times2.05&0.3279\times1.1\\ -0.3607\times2.05+1.1&-0.3607\times1.1+1.2\end{bmatrix}=\begin{bmatrix}0.6721&0.3607\\0.3607&0.8033\end{bmatrix}
$$

### 7.4 读结果

1. **位置**：预测 1.0，测量 1.6，更新后 1.4033，偏向测量（因为 $P_{11}^-=2.05>R=1$）。位置方差从 2.05 降到 0.672。
2. **速度**：我们根本没测速度！但速度估计从 1.0 升到 1.2164，方差从 1.2 降到 0.803。原因是增益 $K_2=0.3607$ 来自协方差的非对角元：位置比预测偏大，说明速度很可能也偏大。
3. **新息检验**：$\nu^2/S=0.36/3.05=0.118$（一维 NIS），远小于 95% 阈值 3.84，说明这次测量与预测很一致。

> **面试要点**：能说出"卡尔曼滤波里的非对角协方差让它能修正没直接测到的状态"，是区分"背公式"和"懂滤波"的标志。

---

## 8. EKF：线性化与雅可比

### 8.1 它要解决什么问题

机器人运动模型是非线性的。例如，独轮车 / 差速 / 自行车模型中位置更新含 $\cos\theta,\sin\theta$。测量模型也常非线性（距离、方位角）。线性卡尔曼滤波的前提不成立。

**EKF 的做法**：在当前估计点附近做一阶泰勒展开，把非线性函数**局部近似为线性**，再套用卡尔曼公式；均值仍用**非线性函数**传播。

### 8.2 模型

$$
x_k=f(x_{k-1},u_k)+w_k,\qquad z_k=h(x_k)+v_k
$$

### 8.3 线性化与雅可比

在 $\hat x_{k-1}$ 处展开 $f$：

$$
f(x_{k-1},u)\approx f(\hat x_{k-1},u)+F_k\,(x_{k-1}-\hat x_{k-1}),\qquad F_k=\frac{\partial f}{\partial x}\Big|_{\hat x_{k-1},u_k}
$$

雅可比矩阵 $F_k$ 的第 $(i,j)$ 个元素是 $\partial f_i/\partial x_j$，表示"第 $j$ 个状态的微小变化，会让下一时刻第 $i$ 个状态变多少"。

类似地，在先验 $\hat x_k^-$ 处展开 $h$：

$$
H_k=\frac{\partial h}{\partial x}\Big|_{\hat x_k^-}
$$

### 8.4 EKF 算法

**预测**

$$
\hat x_k^-=f(\hat x_{k-1},u_k),\qquad P_k^-=F_kP_{k-1}F_k^\top+Q_k
$$

**更新**

$$
\nu_k=z_k-h(\hat x_k^-),\quad S_k=H_kP_k^-H_k^\top+R,\quad K_k=P_k^-H_k^\top S_k^{-1}
$$
$$
\hat x_k=\hat x_k^-+K_k\nu_k,\qquad P_k=(I-K_kH_k)P_k^-
$$

区别只有三点：均值用 $f,h$ 本身；协方差用雅可比 $F,H$；$F,H$ **每一步都要重新计算**。

### 8.5 例：独轮车（unicycle）运动模型的雅可比

状态 $x=[x,\ y,\ \theta]^\top$（位置 m，航向 rad），输入线速度 $v$（m/s）与角速度 $\omega$（rad/s），周期 $\Delta t$。离散模型（欧拉）：

$$
f(x,u)=\begin{bmatrix}x+v\cos\theta\,\Delta t\\ y+v\sin\theta\,\Delta t\\ \theta+\omega\,\Delta t\end{bmatrix}
$$

雅可比对状态求偏导：

$$
F=\frac{\partial f}{\partial x}=\begin{bmatrix}1&0&-v\sin\theta\,\Delta t\\0&1&v\cos\theta\,\Delta t\\0&0&1\end{bmatrix}
$$

读法：第三列表示"航向误差会造成横向位置误差"。航向估计错 $\delta\theta$，行驶距离 $v\Delta t$ 后，位置误差约 $v\Delta t\cdot\delta\theta$，垂直于行驶方向。**这就是为什么航向误差比位置误差更危险——它随行驶距离放大。**

若观测是直接测位置 $z=[x,y]^\top$（例如 AMCL 输出的位姿或 GPS）：

$$
h(x)=\begin{bmatrix}x\\y\end{bmatrix},\qquad H=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}
$$

### 8.6 角度的特殊处理

角度是周期量。新息里的航向差必须归一化到 $(-\pi,\pi]$：

```python
def wrap(a):
    return (a + math.pi) % (2*math.pi) - math.pi
# innovation for yaw: nu_yaw = wrap(z_yaw - yaw_pred)
```

漏掉这一步，航向从 $+179°$ 越过 $\pm180°$ 时，新息变成接近 $360°$，滤波器会瞬间"转一圈"——这是导航里典型的位姿跳变来源之一。

### 8.7 EKF 的局限

- 线性化误差：初始误差大或非线性强时，雅可比在错误的点上计算，可能发散；
- 只能表示单峰高斯：多峰（如位置歧义）无法表达，所以全局定位用粒子滤波（AMCL）；
- 雅可比推导易错：可用数值差分检验。

改进：迭代 EKF、UKF（无迹变换）、误差状态 EKF（ESKF，姿态用误差状态，常见于 IMU 融合）。

---

## 9. 手算三：EKF 预测 + 更新

**设定**（教学数值，单位 m、rad）：独轮车，当前后验 $\hat x_0=[0,0,0]^\top$（在原点朝向 $+x$），$P_0=\operatorname{diag}(0.04,\ 0.04,\ 0.01)$。输入 $v=1$ m/s，$\omega=0$，$\Delta t=1$ s。过程噪声 $Q=\operatorname{diag}(0.01,\ 0.01,\ 0.005)$。随后收到一个位置测量 $z=[1.2,\ 0.3]^\top$（比如地图定位给出的 $x,y$），$R=\operatorname{diag}(0.09,\ 0.09)$。

### 9.1 预测

均值：$\theta=0$，所以 $\hat x^-=[0+1\cdot1,\ 0+0,\ 0]^\top=[1,\ 0,\ 0]^\top$。

雅可比（$\theta=0$：$\sin0=0,\cos0=1$）：

$$
F=\begin{bmatrix}1&0&0\\0&1&1\\0&0&1\end{bmatrix}
$$

计算 $FP_0F^\top$。先算 $FP_0$：

$$
FP_0=\begin{bmatrix}1&0&0\\0&1&1\\0&0&1\end{bmatrix}\begin{bmatrix}0.04&0&0\\0&0.04&0\\0&0&0.01\end{bmatrix}=\begin{bmatrix}0.04&0&0\\0&0.04&0.01\\0&0&0.01\end{bmatrix}
$$

再乘 $F^\top$：

$$
FP_0F^\top=\begin{bmatrix}0.04&0&0\\0&0.05&0.01\\0&0.01&0.01\end{bmatrix}
$$

（$y$ 的方差 $0.04+0.01$：来自航向方差 $0.01$ 被 $v\Delta t=1$ 放大进入横向位置。）

加 $Q$：

$$
P^-=\begin{bmatrix}0.05&0&0\\0&0.06&0.01\\0&0.01&0.015\end{bmatrix}
$$

### 9.2 更新

$H=\begin{bmatrix}1&0&0\\0&1&0\end{bmatrix}$。

新息：$\nu=z-H\hat x^-=[1.2-1,\ 0.3-0]^\top=[0.2,\ 0.3]^\top$。

新息协方差：$HP^-H^\top$ 取 $P^-$ 的左上 $2\times2$ 块 $\begin{bmatrix}0.05&0\\0&0.06\end{bmatrix}$，加 $R$：

$$
S=\begin{bmatrix}0.14&0\\0&0.15\end{bmatrix}
$$

增益 $K=P^-H^\top S^{-1}$：$P^-H^\top$ 取 $P^-$ 的前两列：

$$
P^-H^\top=\begin{bmatrix}0.05&0\\0&0.06\\0&0.01\end{bmatrix},\qquad K=\begin{bmatrix}0.05/0.14&0\\0&0.06/0.15\\0&0.01/0.15\end{bmatrix}=\begin{bmatrix}0.3571&0\\0&0.4\\0&0.0667\end{bmatrix}
$$

状态更新：

$$
\hat x=\hat x^-+K\nu=\begin{bmatrix}1+0.3571\times0.2\\0+0.4\times0.3\\0+0.0667\times0.3\end{bmatrix}=\begin{bmatrix}1.0714\\0.12\\0.02\end{bmatrix}
$$

**注意第三个分量**：我们没有测航向，但航向估计从 0 变到了 0.02 rad（1.1°）。原因是 $P^-$ 里 $y$ 和 $\theta$ 的协方差为 $0.01$：测量说"车在预测的左边 0.3 m"，航向偏左也是可能的原因之一。

协方差更新 $P=(I-KH)P^-$：

- $P_{xx}=(1-0.3571)\times0.05=0.03214$
- $P_{yy}=0.06-0.4\times0.06=0.036$
- $P_{y\theta}=0.01-0.4\times0.01=0.006$
- $P_{\theta\theta}=0.015-0.0667\times0.01=0.01433$

（可以验证：不确定度全部下降。）

### 9.3 新息检验（NIS）

$$
\text{NIS}=\nu^\top S^{-1}\nu=\frac{0.2^2}{0.14}+\frac{0.3^2}{0.15}=0.2857+0.6=0.886
$$

2 自由度、95% 阈值为 5.99；$0.886<5.99$，接受这次测量。如果 $z$ 是 $[3.0,\ 1.5]$，NIS 将远超阈值，应当判为异常值（比如定位发生了跳变）而拒绝。

---

## 10. Q、R 调参与一致性检验

### 10.1 $R$ 怎么定

测量噪声协方差有明确来源：

- 传感器手册给出的噪声标准差，$R=\sigma^2$；
- **更靠谱**：让机器人静止采集一段数据，计算样本方差；
- 若传感器输出自带协方差（如 AMCL 位姿协方差、里程计协方差字段），先检查它是否合理，再决定是否直接使用或乘一个缩放系数。

### 10.2 $Q$ 怎么定

$Q$ 代表"模型没覆盖的东西"，没有现成数据可查，是最难的：

1. 从物理出发：加速度扰动、打滑、轮径误差等，估算它们一个周期内造成的状态变化的方差；
2. 从现象出发：滤波输出**滞后**、对测量反应迟钝 → $Q$ 偏小；输出**抖动**、跟着测量噪声跑 → $Q$ 偏大；
3. 用 NIS / NEES 做一致性检验，调到统计一致；
4. $Q/R$ 的**比值**决定增益大小，绝对数值缩放不改变稳态增益，但改变协方差输出的尺度。

| 症状 | 可能原因 | 调整 |
|---|---|---|
| 估计平滑但明显滞后于测量 | $Q$ 偏小或 $R$ 偏大 | 增大 $Q$ / 减小 $R$ |
| 估计抖动，随测量噪声跳 | $Q$ 偏大或 $R$ 偏小 | 减小 $Q$ / 增大 $R$ |
| 协方差很小但误差很大（过度自信） | $Q,R$ 都偏小 | 增大 |
| 滤波发散 | 模型错误、线性化失效、$Q$ 过小 | 检查模型和雅可比 |

### 10.3 一致性检验

**NEES**（Normalized Estimation Error Squared，需要真值，仿真或带真值设备中使用）：

$$
\epsilon_k=(x_k-\hat x_k)^\top P_k^{-1}(x_k-\hat x_k)\ \sim\ \chi^2_n
$$

**NIS**（Normalized Innovation Squared，不需要真值，实机可用）：

$$
\text{NIS}_k=\nu_k^\top S_k^{-1}\nu_k\ \sim\ \chi^2_m
$$

滤波器"一致"（consistent），意味着它声称的协方差与实际误差统计相符。如果滤波器一致，NIS 的期望等于测量维度 $m$。

**手算（示例）**：$m=2$，连续取 10 个 NIS（共 $20$ 自由度），平均值应落在 $\chi^2_{20}$ 的 95% 区间 $[9.59,\ 34.17]/10=[0.96,\ 3.42]$。

- 平均 $=1.1$：在区间内，一致。
- 平均 $=0.3$：低于下界，滤波器**过于保守**（$Q$ 或 $R$ 偏大）。
- 平均 $=6$：高于上界，滤波器**过于自信**（$Q$ 或 $R$ 偏小，或模型有偏差）。

常用 $\chi^2$ 阈值：

| 自由度 | 95% | 99% |
|---|---|---|
| 1 | 3.84 | 6.63 |
| 2 | 5.99 | 9.21 |
| 3 | 7.81 | 11.34 |

### 10.4 马氏距离门控（gating）

测量要不要进入更新，用马氏距离判断：$d^2=\nu^\top S^{-1}\nu$。若 $d^2>\chi^2_{m,\ 0.99}$，认为是异常值，拒绝（或在 `robot_localization` 里叫 `rejection_threshold`）。门控能防止一次坏测量把状态拉飞；但门限过紧会在滤波器已经跑偏时拒绝掉所有正确测量，导致"锁死"。

### 10.5 其他常见问题

- **可观性**：测量必须使状态可观。只测速度，位置不可观，位置协方差会无限增长。
- **来源相关**：同一信息被重复融合（例如轮速里程计内部已用了 IMU，又单独融合 IMU），会让滤波器过度自信。
- **时间戳**：测量应该用它的**采样时刻**去更新，晚到的测量要回滚或外推（见[里程计与传感器融合](./04_里程计与传感器融合.md)）。
- **协方差设置不当**：很多"ROS 里程计"消息的协方差字段是 0 或随意值，直接喂给 EKF 会让它认为该信息"完美"。

---

## 11. 在本项目里怎么用

CV 对 P5 的描述：EKF/AMCL 定位——融合 IMU、轮式编码器、2D LiDAR 做基于地图的定位，处理时间、坐标系、运动方向不一致造成的漂移和位姿跳变，输出连续的车辆状态。

用本章的语言，典型分工（通用做法）是：

- **EKF**：以运动模型（车辆运动学）+ 轮速 / IMU 数据为主，输出**连续、平滑**的局部状态（`odom` 系）。预测频率可以高于传感器频率。
- **AMCL**：用激光与地图比较，提供**全局位姿**（`map` 系），其结果既可以作为 EKF 的测量，也可以通过 `map→odom` 变换体现。
- EKF 的 $F,H$、$Q,R$ 对应第 8、10 章；"位姿跳变"可以用 NIS 门控与角度归一化预防。

详细的融合方式见 [里程计与传感器融合](./04_里程计与传感器融合.md) 与 [P5 README](../../01_项目课程/P5_车式移动机器人自主导航/README.md)。

> **本项目的 EKF 配置思路（量级，调参时按实机微调）**：P5 的 EKF 状态通常取平面车辆的 $[x,y,\psi,v,\omega]^\top$（也可再加 IMU 陀螺零偏一维），测量输入为轮速 $v$、IMU 偏航角速度 $\omega_z$，以及 AMCL 的位姿（经 `map→odom` 体现）。$Q$ 由输入噪声经 $GQ_uG^\top$ 得到，$R$ 来自静止采样的方差，再按“滤波器过于自信 / 过于迟钝”的现象整体缩放。门控采用马氏距离（NIS）拒绝离群测量。实现上我们使用并配置 `robot_localization` 类 EKF（即“使用并配置”，而不是自己从零实现），自己实现的部分是理解其原理并据此选取状态、噪声与门限。

---

## 12. 常见错误与故障现象

| 现象 | 常见原因 |
|---|---|
| 滤波器发散 | 模型错误、$Q$ 过小、初始协方差过小、线性化点远离真值 |
| 航向在 ±π 附近突然跳变 | 新息航向差没做角度归一化 |
| 位置估计随时间无限漂 | 缺少绝对测量（可观性不足） |
| 估计过度自信，被坏测量拉偏 | 没有门控；$R$ 偏小 |
| 估计迟迟不收敛 | $P_0$ 太小，滤波器不相信新测量 |
| 协方差出现负数 / 非对称 | 直接用 $(I-KH)P$ 的数值误差；改用 Joseph 形式 |
| 融合后比单个传感器还差 | 噪声模型错误、测量相关性被忽略、时间戳不对齐 |
| 倒车时轨迹反向 | 速度符号错误（见里程计章） |

---

## 13. 面试怎么问、怎么答

**Q1. Explain the Kalman filter in plain words.**

> A: It is a predict–correct loop. In the prediction step I use the motion model to move the state forward and I grow the uncertainty. In the correction step I compare a sensor measurement with the prediction, and I blend them with a gain that depends on how uncertain each one is. If the prediction is uncertain and the sensor is precise, the gain is large and I trust the sensor. If the sensor is noisy, the gain is small and I trust the model. After the update the uncertainty shrinks.

中文要点：预测—修正；增益 = 预测不确定度 /（预测 + 测量不确定度）；更新使协方差下降。

**Q2. How does the EKF differ from the Kalman filter?**

> A: The Kalman filter assumes linear models. The EKF handles nonlinear motion or measurement models by linearising them around the current estimate with a Jacobian. I propagate the mean with the nonlinear function, but I propagate the covariance with the Jacobian. The cost is that the linearisation can be inaccurate when the error is large, and I have to compute the Jacobians at every step.

追问：独轮车模型的雅可比是什么？为什么航向误差更危险？（答：$F$ 第三列 $[-v\sin\theta\Delta t,\ v\cos\theta\Delta t,\ 1]^\top$，航向误差随行驶距离放大成横向误差。）

**Q3. How do you tune Q and R?**

> A: I get R from the sensor datasheet or, better, from the sample variance of a stationary recording. Q is harder because it models what the model misses, so I start from physical estimates like wheel slip or acceleration noise and then tune it with the innovation statistics. If the filter lags behind the measurements I increase Q; if it is noisy I decrease it. I check consistency with NIS: the average should be close to the measurement dimension.

**Q4. The filter output jumps occasionally. What do you check?**

> A: First, angle wrapping in the innovation. Second, timestamps and frames of each measurement, since a late or wrongly framed measurement looks like a jump. Third, the measurement covariance, because an over-confident source pulls the state. Fourth, Mahalanobis gating to reject outliers.

追问：门控太紧会怎样？（答：滤波器偏了之后会拒绝所有正确测量，锁死在错误估计上，需要超时重置机制。）

**Q5. Why can a position-only measurement correct velocity?**

> A: Because the covariance has an off-diagonal term between position and velocity. The prediction step creates that correlation, since velocity integrates into position. The gain then distributes part of the position correction to the velocity.

## 14. 本章要点速记

1. 贝叶斯滤波 = 预测 + 更新；卡尔曼滤波 = 线性高斯下的精确闭式解。
2. 预测：$\hat x^-=F\hat x$，$P^-=FPF^\top+Q$；不确定度变大。
3. 更新：$K=P^-H^\top(HP^-H^\top+R)^{-1}$，$\hat x=\hat x^-+K\nu$，$P=(I-KH)P^-$；不确定度变小。
4. $K$ 来自最小化后验协方差的迹；与高斯相乘（精度加权）结果一致。
5. 一维：$K=P^-/(P^-+R)$；非对角协方差让未测量状态也被修正。
6. EKF：均值用非线性函数，协方差用雅可比 $F=\partial f/\partial x$、$H=\partial h/\partial x$。
7. 独轮车 $F$ 的第三列表示航向误差转成横向位置误差。
8. 新息中的角度必须归一化。
9. $R$ 用静止数据估；$Q$ 用物理估计 + NIS 调；$Q/R$ 比值决定增益。
10. NEES / NIS 检验一致性；马氏距离门控剔除异常值。

## 自测题

1. 一维：$P^-=3,\ R=1$，求 $K$ 与 $P$。（答：$K=3/4=0.75$；$P=0.25\times3=0.75$。）
2. 先验 $\hat x^-=10$、$P^-=4$，测量 $z=12$、$R=4$（$H=1$），求后验。（答：$K=0.5$，$\hat x=11$，$P=2$。）
3. 为什么预测步只会让 $P$ 变大，更新步只会让它变小？（要点：预测加了 $Q$；更新加入了信息，$P=(I-KH)P^-\preceq P^-$。）
4. 独轮车 $\theta=90°$，$v=2$ m/s，$\Delta t=0.5$ s，写出 $F$。（答：$-v\sin\theta\Delta t=-1$，$v\cos\theta\Delta t=0$：$F=\begin{bmatrix}1&0&-1\\0&1&0\\0&0&1\end{bmatrix}$。）
5. 2 维测量，新息 $\nu=[0.5,\ 0.5]^\top$，$S=\operatorname{diag}(0.04,\ 0.04)$，是否通过 99% 门控？（答：NIS $=0.25/0.04\times2=12.5>9.21$，拒绝。）
6. 10 个 NIS 的平均值远小于期望维度，说明什么？（要点：过度保守，$Q$ 或 $R$ 偏大。）
7. 为什么同一个 IMU 信息不能既放在里程计里又单独喂给 EKF？（要点：信息重复，被当作独立证据，协方差过小，滤波过度自信。）

## 相关链接

- 上一章：[互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)
- 下一章：[里程计与传感器融合](./04_里程计与传感器融合.md)
- 相关：[粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)、[时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)
- 基础：[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)、[LQR 最优控制](../05_控制理论/04_LQR最优控制.md)（与卡尔曼滤波对偶）
- 项目：[P5 车式移动机器人自主导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)
- 八股：[状态估计与 SLAM 八股](../../03_八股知识/03_机器人学/03_状态估计与SLAM八股.md)
