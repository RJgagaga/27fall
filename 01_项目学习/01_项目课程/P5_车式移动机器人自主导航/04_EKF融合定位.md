# 04 EKF 融合定位

> **一句话定位**：EKF（扩展卡尔曼滤波，Extended Kalman Filter）把"按运动学模型往前推"（预测）和"拿传感器读数去纠正"（更新）交替进行，同时跟踪一个**不确定度（协方差）**。它是 P5 里"连续车辆状态"的来源：高频、平滑、可融合 IMU 与轮速。

**本章学完你能：**

- 讲清预测—更新循环，并从"线性组合的方差"推导出 $P^-=FPF^\top+Q$；
- 说出 $F$、$Q$、$H$、$R$、$K$、$S$ 各是什么、怎么得到、单位是什么；
- 为一个车式模型写出状态、非线性预测函数和雅可比 $F$；
- **手算一个完整的 3 状态例子**：预测协方差、卡尔曼增益、更新状态与协方差；
- 解释为什么只测 $y$ 和 $\psi$ 时 $x$ 的方差不会下降（可观测性）；
- 说明 `robot_localization` 的典型配置思路（哪些量融合、哪些不融合）、如何调 $Q/R$；
- 列出 EKF 的常见失效模式（角度环绕、协方差填 0、门限、延迟测量）。

**前置知识：**

- 卡尔曼滤波与 EKF（基础）：[../../02_基础知识/05_状态估计/03_卡尔曼滤波与EKF.md](../../02_基础知识/05_状态估计/03_卡尔曼滤波与EKF.md)
- 概率统计与高斯分布：[../../02_基础知识/01_数学/03_概率统计与高斯分布.md](../../02_基础知识/01_数学/03_概率统计与高斯分布.md)
- 线性代数（矩阵乘法、转置）：[../../02_基础知识/01_数学/01_线性代数.md](../../02_基础知识/01_数学/01_线性代数.md)
- 本项目：[02 运动学](./02_车辆运动学与里程计.md)、[03 传感器与数据质量](./03_传感器与数据质量_时间坐标与运动方向.md)

**目录**

1. 要解决什么问题
2. 一维直觉：先把"增益"弄懂
3. 多维卡尔曼：状态、预测、更新
4. 为什么是 $P^-=FPF^\top+Q$（逐项讲透）
5. $Q$ 怎么得到
6. 非线性与雅可比：车式模型的 EKF
7. 完整手算（3 状态，预测+更新）
8. 可观测性：为什么 $x$ 的方差没降
9. 异步多传感器、延迟测量与门限
10. robot_localization 的典型配置思路
11. 调参方法与一致性检验
12. 常见错误与故障现象
13. 面试问答 / 要点速记 / 自测题 / 链接

---

## 1. 要解决什么问题

小车有三类信息：

1. **运动模型**：已知上一时刻的位姿、当前速度和角速度，可以推算下一时刻位姿——但有误差（打滑、模型不准）；
2. **传感器**：编码器测速度、陀螺测角速度、激光匹配出位姿——但都有噪声；
3. **不确定度**：我们不只想要一个估计值，还想知道"这个估计有多可信"，以便决定下一条测量该信多少。

卡尔曼滤波的思想：把状态看成**高斯分布** $\mathcal N(\hat x,P)$（均值 + 协方差），预测时让协方差变大（模型不是完美的），更新时用一个加权（卡尔曼增益）把预测与测量混合，并让协方差变小。**谁更可信，权重就更大**。

---

## 2. 一维直觉：先把"增益"弄懂

**问题**：你估计小车的位置在 0，不确定度是方差 $P=4$（标准差 2 m）；现在测得 $z=1$，测量方差 $R=1$（标准差 1 m）。融合后的估计？

加权平均，权重与"可信度（方差倒数）"成正比：

$$K=\frac{P}{P+R}=\frac{4}{5}=0.8,\qquad \hat x^+=\hat x^-+K(z-\hat x^-)=0+0.8\times1=0.8,\qquad P^+=(1-K)P=0.2\times4=0.8.$$

意义：

- 测量更准（$R$ 小）→ $K$ 接近 1 → 更信测量；
- 预测更准（$P$ 小）→ $K$ 接近 0 → 更信预测；
- 融合后方差 $0.8$ 比预测（4）和测量（1）**都小**——两个独立的信息加起来比任何一个都确定。

这一维公式就是多维的缩影，下面把它推广。

---

## 3. 多维卡尔曼：状态、预测、更新

### 3.1 符号

| 符号 | 含义 | 维度 |
|---|---|---|
| $\mathbf x$ | 状态向量（本例 $[x,y,\psi]^\top$） | $n$ |
| $\hat{\mathbf x}^-,\hat{\mathbf x}^+$ | 先验（预测后）/ 后验（更新后）估计 | $n$ |
| $P^-,P^+$ | 先验/后验协方差（$n\times n$，对称正定） | $n\times n$ |
| $\mathbf u$ | 控制输入（本例 $[v,\omega]^\top$，来自轮速和陀螺） | |
| $f(\cdot)$ | 状态转移函数（运动学模型） | |
| $F=\partial f/\partial\mathbf x$ | 状态转移雅可比 | $n\times n$ |
| $Q$ | 过程噪声协方差（模型不确定度） | $n\times n$ |
| $\mathbf z$ | 测量向量 | $m$ |
| $h(\cdot)$ | 测量函数 | |
| $H=\partial h/\partial\mathbf x$ | 测量雅可比 | $m\times n$ |
| $R$ | 测量噪声协方差 | $m\times m$ |
| $\mathbf y=\mathbf z-h(\hat{\mathbf x}^-)$ | 新息（innovation，残差） | $m$ |
| $S=HP^-H^\top+R$ | 新息协方差 | $m\times m$ |
| $K=P^-H^\top S^{-1}$ | 卡尔曼增益 | $n\times m$ |

### 3.2 一个周期的五个公式

**预测**：
$$\hat{\mathbf x}^-=f(\hat{\mathbf x}^+_{k-1},\mathbf u),\qquad P^-=FP^+_{k-1}F^\top+Q.$$

**更新**：
$$\mathbf y=\mathbf z-h(\hat{\mathbf x}^-),\quad S=HP^-H^\top+R,\quad K=P^-H^\top S^{-1},$$
$$\hat{\mathbf x}^+=\hat{\mathbf x}^-+K\mathbf y,\qquad P^+=(I-KH)P^-\ \ \text{(数值上更稳的写法: Joseph 形式 }(I-KH)P^-(I-KH)^\top+KRK^\top).$$

```text
          ┌─────────────── 预测 (时间更新) ───────────────┐
 x+, P+ ─►│ x- = f(x+, u)        P- = F P+ F^T + Q       │─┐
  (k-1)   └────────────────────────────────────────────────┘ │ 不确定度变大
                                                              ▼
          ┌─────────────── 更新 (测量更新) ───────────────┐
  z, R ──►│ y = z - h(x-)   S = H P- H^T + R              │
          │ K = P- H^T S^-1                                │──► x+, P+ (k)
          │ x+ = x- + K y   P+ = (I - K H) P-              │    不确定度变小
          └────────────────────────────────────────────────┘
```

---

## 4. 为什么是 $P^-=FPF^\top+Q$（逐项讲透）

这是你在笔记里连续追问的点："为什么是这个矩阵、为什么是这样的矩阵相乘"。这里从最基本的概率性质讲起。

### 4.1 基本事实：线性组合的方差

对随机变量 $X,Y$ 和常数 $a,b$：
$$\mathrm{Var}(aX+bY)=a^2\mathrm{Var}(X)+b^2\mathrm{Var}(Y)+2ab\,\mathrm{Cov}(X,Y).$$

### 4.2 一个两状态例子：位置+速度

状态 $[p,v]^\top$，离散模型 $p'=p+v\,\Delta t,\ v'=v$。写成矩阵 $F=\begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}$。假设当前 $p$ 与 $v$ 的误差独立，协方差 $P=\mathrm{diag}(\sigma_p^2,\sigma_v^2)$。

用 4.1 的公式逐项算新的协方差：

- $\mathrm{Var}(p')=\mathrm{Var}(p+v\Delta t)=\sigma_p^2+\Delta t^2\sigma_v^2$（因为独立，协方差项为 0）；
- $\mathrm{Var}(v')=\sigma_v^2$；
- $\mathrm{Cov}(p',v')=\mathrm{Cov}(p+v\Delta t,\ v)=\Delta t\,\sigma_v^2$。

写成矩阵：
$$P'=\begin{bmatrix}\sigma_p^2+\Delta t^2\sigma_v^2&\Delta t\,\sigma_v^2\\\Delta t\,\sigma_v^2&\sigma_v^2\end{bmatrix}.$$

再用 $FPF^\top$ 验证：
$$FP=\begin{bmatrix}\sigma_p^2&\Delta t\sigma_v^2\\0&\sigma_v^2\end{bmatrix},\qquad (FP)F^\top=\begin{bmatrix}\sigma_p^2+\Delta t^2\sigma_v^2&\Delta t\sigma_v^2\\\Delta t\sigma_v^2&\sigma_v^2\end{bmatrix}.$$

两者完全相同。**所以 $FPF^\top$ 不是什么神秘公式，它就是"每个新状态都是旧状态的线性组合，按方差公式把所有组合的方差和协方差一次算完"的矩阵写法。** 数学上：若 $\mathbf x'=F\mathbf x$，则 $\mathrm{Cov}(\mathbf x')=F\,\mathrm{Cov}(\mathbf x)\,F^\top$。

### 4.3 为什么出现非对角项

注意新的协方差里出现了 $\Delta t\,\sigma_v^2$：位置误差和速度误差**相关了**——因为新的位置里含有旧的速度。这个相关是"免费的信息"：以后只要测量到速度，位置也会随之被修正。这是卡尔曼滤波能用"间接测量"改善其他状态的原因。

### 4.4 为什么要加 $Q$

模型 $f$ 只是近似：轮子会打滑，加速度会有未建模扰动。把这些未建模的扰动看成额外的、与状态无关的随机噪声 $\mathbf w\sim\mathcal N(0,Q)$：$\mathbf x'=F\mathbf x+\mathbf w$，那么由独立性，协方差再加上 $Q$：$P^-=FPF^\top+Q$。**$Q$ 越大，滤波器越"不信自己的模型"，越听测量的。**

### 4.5 为什么要有转置

$\mathrm{Cov}(F\mathbf x)=\mathbb E[F(\mathbf x-\bar{\mathbf x})(\mathbf x-\bar{\mathbf x})^\top F^\top]$，转置来自 $(F\mathbf d)(F\mathbf d)^\top=F\mathbf d\mathbf d^\top F^\top$。从维度看：$P$ 是 $n\times n$，左乘 $F$ 作用在"行"上，右乘 $F^\top$ 作用在"列"上，各对应协方差的两个下标。

---

## 5. $Q$ 怎么得到

### 5.1 思路：把输入噪声映射到状态空间

如果模型的不确定性来自**输入噪声**（例如轮速 $v$、陀螺 $\omega$ 的测量噪声），设 $\mathbf u=\bar{\mathbf u}+\mathbf n_u$，$\mathbf n_u\sim\mathcal N(0,Q_u)$，则状态噪声：
$$\mathbf w\approx G\mathbf n_u,\quad G=\frac{\partial f}{\partial\mathbf u},\qquad Q=G\,Q_u\,G^\top.$$

这与 4.2 同理：$\mathbf w$ 是 $\mathbf n_u$ 的线性组合，协方差为 $GQ_uG^\top$。

### 5.2 位置—速度模型的离散白噪声加速度

若加速度是白噪声，标准差 $\sigma_a$，则一个周期内 $p$ 增加 $\tfrac12 a\Delta t^2$，$v$ 增加 $a\Delta t$，所以 $G=[\tfrac12\Delta t^2,\ \Delta t]^\top$，
$$Q=\sigma_a^2\begin{bmatrix}\Delta t^4/4&\Delta t^3/2\\\Delta t^3/2&\Delta t^2\end{bmatrix}.$$

> **示例手算（教学数值）**：$\Delta t=0.1,\ \sigma_a=0.5\ \mathrm{m/s^2}$：$G=[0.005,\ 0.1]^\top$，$Q=0.25\times\begin{bmatrix}2.5\times10^{-5}&5\times10^{-4}\\5\times10^{-4}&0.01\end{bmatrix}=\begin{bmatrix}6.25\times10^{-6}&1.25\times10^{-4}\\1.25\times10^{-4}&2.5\times10^{-3}\end{bmatrix}$。

### 5.3 车式小车的 $Q$

下面的 3 状态例子中，$Q_u=\mathrm{diag}(\sigma_v^2,\sigma_\omega^2)$。$\sigma_v$ 反映轮速测量噪声和打滑的随机部分，$\sigma_\omega$ 反映陀螺噪声。数值来源（典型做法）：

1. 传感器静止/匀速记录，算方差（噪声下限）；
2. 加上模型误差余量（打滑、转向映射误差）——通常需要根据 NIS 一致性检验（第 11 节）上调；
3. 数据手册（ARW 等）作初值。

**$Q$ 不是随便调的"魔法数"，而是"你对模型失准程度的陈述"。**

---

## 6. 非线性与雅可比：车式模型的 EKF

### 6.1 为什么是"扩展"

运动学模型含 $\cos\psi,\sin\psi$，是非线性的，高斯经过非线性变换不再是高斯。EKF 的做法：在当前估计点做一阶泰勒展开，用**雅可比矩阵**当作局部线性模型：

$$f(\mathbf x)\approx f(\hat{\mathbf x})+F(\mathbf x-\hat{\mathbf x}),\qquad F_{ij}=\frac{\partial f_i}{\partial x_j}\Big|_{\hat{\mathbf x}}.$$

均值用**非线性函数**推，协方差用**雅可比**推。

### 6.2 3 状态模型

状态 $\mathbf x=[x,\ y,\ \psi]^\top$，输入 $\mathbf u=[v,\ \omega]^\top$（$v$ 来自轮速，$\omega$ 来自陀螺），欧拉离散：

$$f=\begin{bmatrix}x+v\cos\psi\,\Delta t\\y+v\sin\psi\,\Delta t\\\psi+\omega\Delta t\end{bmatrix}.$$

逐元素求偏导：

$$F=\frac{\partial f}{\partial\mathbf x}=\begin{bmatrix}1&0&-v\sin\psi\,\Delta t\\0&1&v\cos\psi\,\Delta t\\0&0&1\end{bmatrix},\qquad G=\frac{\partial f}{\partial\mathbf u}=\begin{bmatrix}\cos\psi\,\Delta t&0\\\sin\psi\,\Delta t&0\\0&\Delta t\end{bmatrix}.$$

**怎么读 $F$**：第 3 列表示"航向误差如何变成位置误差"。若 $\psi=0$，$F_{23}=v\Delta t$：航向多错 1 rad，走 $v\Delta t$ 后 $y$ 就多错 $v\Delta t$。这就是"航向误差决定横向漂移"的数学表述。

### 6.3 带陀螺零偏的 5 状态版本（常见扩展）

状态 $[x,y,\psi,v,b_\omega]^\top$：陀螺作为输入 $\omega_m$，$\dot\psi=\omega_m-b_\omega$；加速度计（或常速模型）更新 $v$；零偏近似随机游走。

$$F=\begin{bmatrix}1&0&-v\sin\psi\,\Delta t&\cos\psi\,\Delta t&0\\0&1&v\cos\psi\,\Delta t&\sin\psi\,\Delta t&0\\0&0&1&0&-\Delta t\\0&0&0&1&0\\0&0&0&0&1\end{bmatrix}.$$

测量可以有：轮速 $z_v=v$（$H=[0,0,0,1,0]$）、AMCL 位姿 $[x,y,\psi]$（$H=[I_3,\ 0]$）等。零偏是否可观取决于是否有航向的外部参考——AMCL 的航向修正让零偏可观。

---

## 7. 完整手算（3 状态，预测 + 更新）

以下全部为**教学示例数值**。

### 7.1 设定

- 时间步 $\Delta t=0.1\ \mathrm s$；当前后验估计 $\hat{\mathbf x}=[0,\ 0,\ 0]^\top$（位置原点、航向 0）；
- 初始协方差 $P=\mathrm{diag}(0.04,\ 0.04,\ 0.01)$（即 $\sigma_x=\sigma_y=0.2\ \mathrm m$，$\sigma_\psi=0.1\ \mathrm{rad}$）；
- 输入 $v=1\ \mathrm{m/s},\ \omega=0$；
- 输入噪声 $\sigma_v=0.1\ \mathrm{m/s}$，$\sigma_\omega=0.05\ \mathrm{rad/s}$；另加横向模型误差（打滑）$\sigma_{y,slip}=0.005\ \mathrm m$/步。

### 7.2 预测均值

$\hat x^-=0+1\cdot\cos0\cdot0.1=0.1,\ \hat y^-=0,\ \hat\psi^-=0$，即 $\hat{\mathbf x}^-=[0.1,\ 0,\ 0]^\top$。

### 7.3 雅可比

$\psi=0$：$-v\sin\psi\Delta t=0$，$v\cos\psi\Delta t=0.1$，所以
$$F=\begin{bmatrix}1&0&0\\0&1&0.1\\0&0&1\end{bmatrix},\qquad G=\begin{bmatrix}0.1&0\\0&0\\0&0.1\end{bmatrix}.$$

### 7.4 计算 $FPF^\top$

先算 $FP$（$F$ 的每一行与 $P$ 相乘；$P$ 是对角阵，相当于 $F$ 的每一列乘以对应的 $P_{jj}$）：
$$FP=\begin{bmatrix}0.04&0&0\\0&0.04&0.1\times0.01\\0&0&0.01\end{bmatrix}=\begin{bmatrix}0.04&0&0\\0&0.04&0.001\\0&0&0.01\end{bmatrix}.$$

$F^\top=\begin{bmatrix}1&0&0\\0&1&0\\0&0.1&1\end{bmatrix}$（注意：$F_{23}=0.1$ 转置后到了第 3 行第 2 列）。再乘：

- 第 1 行 $[0.04,0,0]$：结果 $[0.04,\ 0,\ 0]$；
- 第 2 行 $[0,\ 0.04,\ 0.001]$：第 2 列 $=0.04\times1+0.001\times0.1=0.0401$，第 3 列 $=0.001$，第 1 列 $0$；
- 第 3 行 $[0,0,0.01]$：第 2 列 $=0.01\times0.1=0.001$，第 3 列 $=0.01$。

$$FPF^\top=\begin{bmatrix}0.04&0&0\\0&0.0401&0.001\\0&0.001&0.01\end{bmatrix}.$$

**读数**：$P_{22}$ 从 0.04 变成 0.0401，增加了 $(v\Delta t)^2\sigma_\psi^2=0.1^2\times0.01=10^{-4}$——航向不确定度泄漏进了 $y$；$P_{23}=0.001$ 是新出现的 $y$–$\psi$ 相关（$v\Delta t\cdot\sigma_\psi^2$）。

### 7.5 计算 $Q$

$Q_u=\mathrm{diag}(0.1^2,\ 0.05^2)=\mathrm{diag}(0.01,\ 0.0025)$。
$$GQ_uG^\top=\begin{bmatrix}0.1^2\times0.01&0&0\\0&0&0\\0&0&0.1^2\times0.0025\end{bmatrix}=\mathrm{diag}(1\times10^{-4},\ 0,\ 2.5\times10^{-5}).$$
注意 $y$ 方向在 $\psi=0$ 时没有输入噪声通道，加上打滑项 $0.005^2=2.5\times10^{-5}$：
$$Q=\mathrm{diag}(10^{-4},\ 2.5\times10^{-5},\ 2.5\times10^{-5}).$$

### 7.6 先验协方差

$$P^-=FPF^\top+Q=\begin{bmatrix}0.0401&0&0\\0&0.040125&0.001\\0&0.001&0.010025\end{bmatrix}.$$

### 7.7 测量更新

假设 AMCL 给出 $y$ 和 $\psi$ 的测量（它的 $x$ 先不用）：$\mathbf z=[y_m,\ \psi_m]^\top=[0.03,\ 0.02]^\top$，
$$H=\begin{bmatrix}0&1&0\\0&0&1\end{bmatrix},\qquad R=\mathrm{diag}(0.0025,\ 0.0004)\ (\sigma_y=5\ \mathrm{cm},\ \sigma_\psi=0.02\ \mathrm{rad}).$$

**新息**：$\mathbf y=\mathbf z-H\hat{\mathbf x}^-=[0.03-0,\ 0.02-0]^\top=[0.03,\ 0.02]^\top$。

**新息协方差**：$HP^-H^\top$ 取出 $P^-$ 的第 2、3 行和列：
$$S=\begin{bmatrix}0.040125+0.0025&0.001\\0.001&0.010025+0.0004\end{bmatrix}=\begin{bmatrix}0.042625&0.001\\0.001&0.010425\end{bmatrix}.$$

行列式 $\det S=0.042625\times0.010425-0.001^2=4.4437\times10^{-4}-10^{-6}=4.4337\times10^{-4}$。逆：
$$S^{-1}=\frac{1}{4.4337\times10^{-4}}\begin{bmatrix}0.010425&-0.001\\-0.001&0.042625\end{bmatrix}\approx\begin{bmatrix}23.513&-2.2555\\-2.2555&96.141\end{bmatrix}.$$

**增益**：$K=P^-H^\top S^{-1}$。$P^-H^\top$ 是 $P^-$ 的第 2、3 列：
$$P^-H^\top=\begin{bmatrix}0&0\\0.040125&0.001\\0.001&0.010025\end{bmatrix}.$$

- 第 1 行（$x$）：$[0,0]$，所以 $K$ 的 $x$ 行为 0（$P^-$ 里 $x$ 与 $y,\psi$ 不相关，这些测量无法改善 $x$）；
- 第 2 行（$y$）：$[0.040125\times23.513+0.001\times(-2.2555),\ 0.040125\times(-2.2555)+0.001\times96.141]=[0.9412,\ 0.0056]$；
- 第 3 行（$\psi$）：$[0.001\times23.513+0.010025\times(-2.2555),\ 0.001\times(-2.2555)+0.010025\times96.141]=[0.0009,\ 0.9616]$。

$$K\approx\begin{bmatrix}0&0\\0.9412&0.0056\\0.0009&0.9616\end{bmatrix}.$$

**直觉核对**：$y$ 的先验方差 0.040（σ=20 cm）远大于测量方差 0.0025（σ=5 cm），所以更信测量，增益 0.94；$\psi$ 同理（先验 0.010，测量 0.0004），增益 0.96。

**状态更新**：$\hat{\mathbf x}^+=\hat{\mathbf x}^-+K\mathbf y$：
- $\Delta x=0$；
- $\Delta y=0.9412\times0.03+0.0056\times0.02=0.02824+0.00011=0.0284$；
- $\Delta\psi=0.0009\times0.03+0.9616\times0.02=0.00003+0.01923=0.0193$。

$$\hat{\mathbf x}^+=[0.1,\ 0.0284,\ 0.0193]^\top.$$

**协方差更新**：$P^+=P^--KHP^-$。$HP^-$ 是 $P^-$ 的第 2、3 行：
$$HP^-=\begin{bmatrix}0&0.040125&0.001\\0&0.001&0.010025\end{bmatrix},$$
$$KHP^-=\begin{bmatrix}0&0&0\\0&0.9412\times0.040125+0.0056\times0.001&0.9412\times0.001+0.0056\times0.010025\\0&0.0009\times0.040125+0.9616\times0.001&0.0009\times0.001+0.9616\times0.010025\end{bmatrix}\approx\begin{bmatrix}0&0&0\\0&0.03777&0.00099\\0&0.00100&0.00964\end{bmatrix}.$$

$$P^+\approx\begin{bmatrix}0.0401&0&0\\0&0.00235&\approx0\\0&\approx0&0.000385\end{bmatrix}.$$

**结果解读**：$\sigma_y$ 从 $\sqrt{0.040125}=20.0\ \mathrm{cm}$ 降到 $\sqrt{0.00235}=4.8\ \mathrm{cm}$（略好于测量的 5 cm）；$\sigma_\psi$ 从 $0.100$ 降到 $\sqrt{0.000385}=0.0196\ \mathrm{rad}$（略好于测量的 0.02）。$\sigma_x$ 仍是 $\sqrt{0.0401}=20\ \mathrm{cm}$。

**马氏距离（门限检查）**：$d^2=\mathbf y^\top S^{-1}\mathbf y=0.03^2(23.513)+2(0.03)(0.02)(-2.2555)+0.02^2(96.141)=0.02116-0.00271+0.03846=0.0569$。2 自由度的 95% 卡方分位数是 5.99，远小于它，所以这条测量**被接受**。

---

## 8. 可观测性：为什么 $x$ 的方差没降

上例中 $P^+_{11}=0.0401$ 没变，这不是算错：测量只涉及 $y,\psi$，而 $x$ 与它们在先验里不相关，所以 $y,\psi$ 的测量**无法提供 $x$ 的信息**。后果与应对：

- 车沿 $+x$ 方向行驶时，$x$（纵向）误差只有当测量含 $x$ 信息时才下降。对激光定位来说，**长直走廊**是典型例子：墙平行于行驶方向，激光不能约束沿走廊方向的位置；
- 但如果车的航向不是 0，$y$、$\psi$ 的测量会通过 $P_{xy}$、$P_{x\psi}$ 间接改善 $x$（相关项非零）；
- 实践意义：**先验里的相关项是滤波器最重要的资产之一**，对角化协方差（丢掉相关项）会损失这种间接改善；
- 协方差在无测量时按 $FPF^\top+Q$ 不断增长——不确定度的增长速度也告诉你"多久必须有一次全局修正"。

---

## 9. 异步多传感器、延迟测量与门限

### 9.1 异步更新

IMU 200 Hz、轮速 50 Hz、AMCL 5–10 Hz，各传感器的更新是**按各自时间戳异步**完成的：每来一条消息，滤波器先预测到这条消息的时间，再做对应的更新。协方差和增益每次独立计算。

### 9.2 延迟测量

AMCL 的位姿时间戳是激光扫描时间，到达 EKF 时已比当前状态晚 50–150 ms。把它当"当前时刻"的测量会出错（见 [03](./03_传感器与数据质量_时间坐标与运动方向.md)）。典型处理：滤波器保存一小段状态与协方差历史，延迟测量到来时**回滚到该时间点、做更新、再重放此后的输入**（`robot_localization` 的做法即是维护历史并回放）。

### 9.3 门限与离群点

- 马氏距离门限：$d^2>\chi^2_{m,\alpha}$ 拒绝。自由度 $m=1$：95% 为 3.84，99% 为 6.63；$m=2$：5.99 / 9.21；$m=3$：7.81 / 11.34；
- 在 `robot_localization` 里对应 `*_pose_rejection_threshold`、`*_twist_rejection_threshold`（以马氏距离为单位）；
- 门限太紧会拒绝所有有用修正，太松挡不住脏数据，一般配合"连续 N 次拒绝则重置/报警"。

### 9.4 角度环绕

任何涉及 $\psi$ 的新息都必须归一化：$y_\psi=\mathrm{wrap}(z_\psi-\hat\psi^-)$，否则在 $\pm\pi$ 附近会出现 $2\pi$ 的"假新息"，K 把它乘进去，位姿瞬间旋转。这是 EKF 实现里最常见的 bug 之一（[03 章 6.3](./03_传感器与数据质量_时间坐标与运动方向.md)）。

---

## 10. robot_localization 的典型配置思路

`robot_localization` 是 ROS 里常用的开源 EKF/UKF 实现（`ekf_localization_node`）。CV 只写了 "EKF"，项目具体用的是否为此包，以及配置细节，请自行确认【待你补全】。下面讲"典型配置思路"，**数值为教学示例**。

### 10.1 状态与配置矩阵

它的状态是 15 维：$[x,y,z,\ \text{roll},\text{pitch},\text{yaw},\ v_x,v_y,v_z,\ \omega_x,\omega_y,\omega_z,\ a_x,a_y,a_z]$。每个传感器有一个 15 个布尔值的 `*_config`，决定"这个传感器的哪些量被送进滤波器"。

### 10.2 车式小车的典型选择

| 传感器 | 融合哪些量 | 不融合哪些 | 理由 |
|---|---|---|---|
| 轮速里程计 | $v_x$（可再加 $\omega_z$） | 位置 $x,y$、$y$ 方向速度 | 位置是积分量，漂移；融合速度更稳。车式小车无横向速度（非完整约束）——可作为 $v_y=0$ 的虚拟测量 |
| IMU | $\omega_z$（偏航角速度），可选 $a_x$ | 绝对 yaw（若无磁力计） | 陀螺最可靠；无磁力计的 yaw 是积分来的，不应当作绝对测量 |
| 全局位姿（AMCL，若融合） | $x,y,\text{yaw}$ | — | 只在"地图系"EKF 里使用 |

### 10.3 YAML 示例（教学示例，非项目配置）

```yaml
# ekf_odom: continuous odom -> base_link
frequency: 50
sensor_timeout: 0.1
two_d_mode: true
publish_tf: true
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom

odom0: /wheel_odom
odom0_config: [false, false, false,   # x y z
               false, false, false,   # roll pitch yaw
               true,  false, false,   # vx vy vz
               false, false, false,   # vroll vpitch vyaw
               false, false, false]   # ax ay az
odom0_queue_size: 10
odom0_twist_rejection_threshold: 5.0

imu0: /imu/data
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, true,     # vyaw from gyro
              true,  false, false]    # ax (optional)
imu0_remove_gravitational_acceleration: true
imu0_queue_size: 20
# process_noise_covariance / initial_estimate_covariance: 15x15, set from calibration
```

要点：`two_d_mode: true` 把 $z$、roll、pitch 及其导数固定为 0；`world_frame: odom` 表示该滤波器在里程系工作，发布 `odom→base_link`；若再启一个 `world_frame: map` 的滤波器并输入 AMCL 位姿，则发布 `map→odom`（双 EKF 结构）。

### 10.4 两种常见系统架构

| 架构 | `odom→base_link` | `map→odom` | 特点 |
|---|---|---|---|
| A（最常见） | EKF（里程+IMU） | AMCL | AMCL 自己算 `map→odom`，简单、通用 |
| B | EKF-1（里程+IMU） | EKF-2（再加入 AMCL/全局位姿） | 全局修正也经过滤波，可加门限、可做平滑 |

CV 写的是 "EKF/AMCL localisation ... output a continuous vehicle state"。项目具体用的是 A 还是 B【待你补全】。两种架构的"连续性"论证见 [06](./06_EKF与AMCL协同_TF分工与连续状态.md)。

### 10.5 参数的含义速查

| 参数 | 含义 | 影响 |
|---|---|---|
| `frequency` | 滤波器输出频率 | 太低则控制器取到旧状态 |
| `sensor_timeout` | 多久没数据就只做预测 | 防止传感器断流后"冻住" |
| `two_d_mode` | 平面模式 | 防止 z/roll/pitch 噪声污染 |
| `*_differential` | 把位姿测量当相邻差分 | 抗绝对漂移，但会积累误差 |
| `*_relative` | 相对于第一条测量 | 初始偏移统一 |
| `*_queue_size` | 缓冲区 | 太小会丢消息，太大增延迟 |
| `process_noise_covariance` | $Q$ | 越大越信传感器 |
| `initial_estimate_covariance` | 初始 $P_0$ | 太小则一开始不肯接受修正 |
| `*_rejection_threshold` | 马氏距离门限 | 离群点保护 |

---

## 11. 调参方法与一致性检验

### 11.1 先测 $R$，再调 $Q$

1. $R$：静止采集一段数据，计算各传感器噪声方差（陀螺静止方差、编码器零速噪声）；
2. $Q$：从模型误差估计：直线/圆周实验里，预测与测量的残差统计；
3. 初值：$P_0$ 取比实际稍大，使滤波器一开始愿意接受修正。

### 11.2 NIS 检验

归一化新息平方 $\mathrm{NIS}=\mathbf y^\top S^{-1}\mathbf y$。若滤波器一致，NIS 服从 $\chi^2_m$，期望等于测量维数 $m$。

- 长期均值 $\gg m$ → 过于自信（$Q$ 或 $R$ 偏小）；
- 长期均值 $\ll m$ → 过于保守（$Q$ 或 $R$ 偏大）。

> **示例**：2 维测量，NIS 的滑动平均长期为 7，远大于 2 → 说明估计过于自信，需要加大 $Q$ 或检查数据是否有系统性错误（时间偏差、坐标）。第 7 节算出的单步 $d^2=0.057$ 很小，说明这一步一致（但单步不能下结论，要看统计）。

### 11.3 现象—调整对照

| 现象 | 可能原因 | 调整 |
|---|---|---|
| 输出平滑但滞后、不跟 AMCL | $Q$ 偏小或 $R$ 偏大 | 增大 $Q$ / 减小 $R$ |
| 输出抖，紧跟噪声 | $Q$ 偏大 | 减小 $Q$ |
| 直行时 yaw 慢慢偏 | 陀螺零偏 | 估计零偏/加 yaw 外部参考 |
| 倒车后位姿突变 | 方向符号 | 检查符号链路 |
| 协方差持续变大 | 没有有效的修正测量 | 检查 AMCL/测量输入、门限 |

---

## 12. 常见错误与故障现象

1. **协方差填 0**：驱动没填，EKF 当作"绝对可信"，该源占满权重，一旦这个源有误差，输出就跟着错；
2. **角度未归一化**：$\pm\pi$ 处瞬间转动；
3. **把积分量当绝对量融合**：直接融合里程计的位置 $(x,y)$ 和 yaw，会把漂移当真值，且与 AMCL 冲突；
4. **坐标系/`child_frame_id` 错**：速度在世界系却当车体系用；
5. **频率与队列**：队列太小导致数据丢失，延迟被累积；
6. **两个节点发布 `odom→base_link`**：EKF 与底盘驱动都发 TF，导致闪烁；
7. **线性化点太差**：初始位姿离真值很远，一阶近似失效，滤波器发散。

---

## 13. 面试问答 / 要点速记 / 自测题 / 链接

### 面试问答

**Q1. Explain how an EKF works, in your own words.**

> An EKF alternates two steps. In the prediction step it pushes the state forward with the vehicle motion model and grows the covariance with $P^-=FPF^\top+Q$, because the model is imperfect. In the update step it compares a sensor reading with the prediction, computes the Kalman gain from the two uncertainties, and corrects the state and shrinks the covariance. "Extended" means I linearise the nonlinear model with a Jacobian around the current estimate.

*追问*：Why $FPF^\top$? —— 状态的新值是旧值的线性组合，协方差按线性组合规则传播；转置来自双线性。*追问*：What happens if $Q$ is too small? —— 过于自信，对测量反应迟钝，位姿滞后、易发散。

**Q2. What did you fuse in the EKF and why not fuse everything?**

> I fused wheel speed and IMU yaw rate for the continuous state, and used the LiDAR-based map pose as the global correction. I avoided fusing integrated quantities, such as odometry position or an unreferenced yaw, as absolute measurements, because their drift would be treated as truth and fight with the global correction.
> 【待你补全：项目里实际融合了哪些量、频率、是否使用 robot_localization】

**Q3. How do you choose Q and R?**

> $R$ from sensor statistics: I log the sensor at rest and compute the variance. $Q$ reflects how wrong the model can be; I start from the input noise mapped through the Jacobian, then adjust it using the innovation consistency (NIS) so that it matches the expected chi-square value. I do not tune by feel alone.
> 【待你补全：实机 EKF 噪声量级（见 README 自查表）】

**Q4. Walk me through a small numerical example.**

> Take a car at the origin heading along x, speed 1 m/s, step 0.1 s. The heading uncertainty is 0.1 rad. After prediction the y variance grows by $(v\Delta t)^2\sigma_\psi^2$ and y becomes correlated with heading. If an absolute pose measurement of y and yaw arrives, the gain is close to one for both because the prior is much less certain than the measurement; x does not improve because the measurement carries no x information.

### 本章要点速记

1. EKF = 预测（模型推进、协方差变大）+ 更新（测量纠正、协方差变小）。
2. $P^-=FPF^\top+Q$：线性组合的协方差传播 + 过程噪声。
3. $F$ 是模型的雅可比：车式模型里 $F_{13}=-v\sin\psi\Delta t,\ F_{23}=v\cos\psi\Delta t$。
4. $Q=GQ_uG^\top$：输入噪声经 $G$ 映射到状态噪声。
5. $K=P^-H^\top S^{-1}$：谁更不确定谁听另一个的；融合后方差比两者都小。
6. 没有被测量覆盖的方向，方差不会下降（可观测性）。
7. 角度新息必须归一化；协方差不能填 0；积分量不当绝对量融合。
8. NIS 一致性检验用来调 $Q/R$。
9. 马氏距离门限做离群点保护：2 自由度 95% 为 5.99。

### 自测题

1. 状态 $[p,v]$，$\Delta t=0.2$，$P=\mathrm{diag}(1,0.25)$，求 $FPF^\top$。（答：$\begin{bmatrix}1+0.04\times0.25&0.2\times0.25\\0.05&0.25\end{bmatrix}=\begin{bmatrix}1.01&0.05\\0.05&0.25\end{bmatrix}$）
2. 一维：$P=9,\ R=1$，$z-\hat x^-=2$，求 $K$、$\hat x^+-\hat x^-$、$P^+$。（答：$K=0.9$，修正 1.8，$P^+=0.9$）
3. 为什么陀螺零偏需要外部参考才可观？
4. 写出 $\psi=\pi/2$ 时 $F$ 的第三列。（答：$[-v\Delta t,\ 0,\ 1]^\top$，因 $\sin(\pi/2)=1,\cos=0$。）
5. 若 NIS 的长期平均是测量维数的 3 倍，应该怎样处理？

### 相关链接

- 上一章：[03 传感器与数据质量](./03_传感器与数据质量_时间坐标与运动方向.md)　下一章：[05 AMCL 粒子滤波定位](./05_AMCL粒子滤波定位.md)
- 基础：[../../02_基础知识/05_状态估计/04_里程计与传感器融合.md](../../02_基础知识/05_状态估计/04_里程计与传感器融合.md)
- 学习笔记（你当时的提问）：[../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md](../../05_学习笔记归档/P5/P5_本对话问答逐字稿.md)
- 返回：[README](./README.md)
