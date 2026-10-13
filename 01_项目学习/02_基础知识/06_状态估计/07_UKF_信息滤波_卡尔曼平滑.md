# UKF、信息滤波与卡尔曼平滑

> **一句话定位**：EKF 把非线性函数“一阶展开”，换来简单，也换来偏差；无迹卡尔曼滤波（UKF）改为“取几个聪明的点，让非线性函数自己去传播均值和协方差”，不用求雅可比；信息滤波把协方差换成它的逆，使多传感器融合变成加法；卡尔曼平滑则利用“未来的测量”回头修正过去的估计，适合离线后处理和带延迟的在线估计。

**本章学完你能：**

- 指出 EKF 线性化为什么会产生均值偏差，并用 Monte Carlo 数据量化；
- 推导无迹变换（UT）的 sigma 点与权重，手算极坐标到直角坐标的传播，与 EKF 和 Monte Carlo 对照；
- 写出 UKF 的预测—更新步骤，说明 $\alpha,\beta,\kappa$ 的作用与负权重现象，以及角度变量需要圆均值；
- 写出信息形式滤波的更新（$Y^+=Y^-+H^\top R^{-1}H$，$y^+=y^-+H^\top R^{-1}z$），手算两个传感器的融合并与顺序卡尔曼更新对照；
- 推导 RTS（Rauch–Tung–Striebel）平滑器的后向递推，手算 4 步标量例子，看到平滑方差比滤波方差小；
- 解释固定滞后平滑的“延迟—精度”折中，给出方差随滞后步数下降并趋于饱和的数值；
- 用英文回答 "UKF vs EKF / why information form / what is a smoother" 等问题。

**前置知识**：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)（预测/更新五公式、雅可比、Q/R）、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)（高斯的线性变换、条件分布）、[数值线性代数与稀疏求解](../01_数学基础/08_数值线性代数与稀疏求解.md)（Cholesky 分解）。

## 目录

1. 问题：EKF 的线性化偏差
2. 符号
3. 无迹变换（UT）
4. 手算：极坐标到直角坐标
5. UKF 算法
6. UKF 的参数与失效模式
7. 信息形式滤波
8. 卡尔曼平滑：RTS
9. 固定滞后平滑
10. 在本项目里怎么用
11. 常见错误与故障现象
12. 面试怎么问、怎么答
13. 本章要点速记
14. 自测题
15. 相关链接

---

## 1. 问题：EKF 的线性化偏差

EKF 的做法是对非线性函数 $g(x)$ 在均值处做一阶泰勒展开：$g(x)\approx g(\mu)+G(x-\mu)$，然后按线性高斯系统的公式传播均值和协方差。这带来两个系统性问题。

**第一，均值偏差。** 若 $g$ 有曲率，随机变量经过 $g$ 之后的均值并不等于 $g(\mu)$。一阶展开丢掉了二阶项，均值被忽略的偏差大小正比于二阶导数乘以协方差。

**第二，协方差偏差。** 同样因为只保留一阶项，输出协方差也被低估或扭曲。

**第三，要求雅可比。** 手写、调试雅可比耗时且易错；有些模型（分段函数、黑箱仿真）根本没有解析雅可比。

**例子（后面详细手算）**：雷达测得距离 $r=1$ m、方位 $\theta=45^\circ$，方位噪声 $15^\circ$。真实转换到直角坐标 $x=r\cos\theta$ 的均值是 $0.6833$ m（Monte Carlo 与解析解一致），而 EKF 一阶近似给出 $0.7071$ m，偏高 $3.5\%$，协方差也不对。UKF 不用雅可比，用几个确定性采样点就把这个均值和协方差算得更准。

## 2. 符号

| 符号 | 含义 |
|---|---|
| $n$ | 状态维数 |
| $\mu,P$ | 状态均值、协方差 |
| $\mathcal X_i$ | 第 $i$ 个 sigma 点，$i=0,\dots,2n$ |
| $W_i^{(m)},W_i^{(c)}$ | 均值权重、协方差权重 |
| $\lambda=\alpha^2(n+\kappa)-n$ | 缩放参数 |
| $\alpha,\beta,\kappa$ | UT 的调节参数 |
| $g(\cdot)$ | 非线性函数（状态转移或观测） |
| $Y=P^{-1}$ | 信息矩阵 |
| $y=Yx$ | 信息向量 |
| $C_k$ | RTS 后向增益 |
| $\hat x_{k\vert j},P_{k\vert j}$ | 利用到时刻 $j$ 的测量得到的 $k$ 时刻估计及协方差 |

## 3. 无迹变换（UT）

### 3.1 思想

“近似一个概率分布，比近似一个非线性函数更容易。”无迹变换不去线性化 $g$，而是：（1）按分布的均值和协方差，**确定性地**挑选一小组 sigma 点；（2）让每个 sigma 点通过真实的非线性 $g$；（3）用加权求和得到输出的均值和协方差。

### 3.2 sigma 点与权重（Julier 形式）

对 $n$ 维高斯 $\mathcal N(\mu,P)$，设 $L$ 满足 $LL^\top=(n+\kappa)P$（Cholesky 分解），取 $2n+1$ 个点：

$$\mathcal X_0=\mu,\quad \mathcal X_i=\mu+L_{:,i},\quad \mathcal X_{n+i}=\mu-L_{:,i},\ \ i=1,\dots,n,$$

权重

$$W_0=\frac{\kappa}{n+\kappa},\qquad W_i=\frac1{2(n+\kappa)},\ \ i=1,\dots,2n.$$

### 3.3 为什么对

- **权重和为 1**：$\frac\kappa{n+\kappa}+2n\cdot\frac1{2(n+\kappa)}=1$；
- **均值还原**：对称点相加，$\sum W_i\mathcal X_i=\mu$；
- **协方差还原**：$\sum W_i(\mathcal X_i-\mu)(\mathcal X_i-\mu)^\top=2\cdot\frac1{2(n+\kappa)}\sum_iL_{:,i}L_{:,i}^\top=\frac{(n+\kappa)P}{n+\kappa}=P$；
- **更高阶**：对称选点使所有奇数阶矩为零（与高斯一致），$\kappa=3-n$ 还可匹配四阶矩的一部分（对标量高斯 $\kappa=2$）。

关键性质：对任意**线性**函数，UT 得到精确结果；对**非线性**函数，UT 的均值和协方差对泰勒展开的精度至少到**二阶**（均值）和**二阶**（协方差，取决于 $\kappa$），而 EKF 对均值只到一阶。

### 3.4 缩放形式

为了在高维空间中避免 sigma 点离均值太远、也避免负权重，常用缩放 UT：

$$\lambda=\alpha^2(n+\kappa)-n,\quad \mathcal X_i=\mu\pm\big[\sqrt{(n+\lambda)P}\big]_i,$$
$$W_0^{(m)}=\frac\lambda{n+\lambda},\quad W_0^{(c)}=\frac\lambda{n+\lambda}+(1-\alpha^2+\beta),\quad W_i^{(m)}=W_i^{(c)}=\frac1{2(n+\lambda)}.$$

$\alpha\in(10^{-4},1]$ 控制 sigma 点散布（小则近）；$\beta=2$ 对高斯最优；$\kappa$ 常取 $0$ 或 $3-n$。

## 4. 手算：极坐标到直角坐标

### 4.1 问题

测量 $(r,\theta)$：均值 $(1\ \text{m},\ 45^\circ)$，$\sigma_r=0.1$ m，$\sigma_\theta=15^\circ=0.2618$ rad，互相独立（示例，非项目实测）。变换 $g(r,\theta)=(r\cos\theta,\ r\sin\theta)$。

### 4.2 三种方法

**(a) Monte Carlo（基准）**：200 万个样本，输出均值 $(0.6834,\ 0.6832)$，协方差 $\begin{bmatrix}0.03815&-0.02660\\-0.02660&0.03818\end{bmatrix}$。解析均值 $E[x]=r\,e^{-\sigma_\theta^2/2}\cos\theta=0.6833$。

**(b) EKF 线性化**：均值直接 $g(\mu)=(0.7071,\ 0.7071)$；雅可比 $J=\begin{bmatrix}\cos\theta&-r\sin\theta\\\sin\theta&r\cos\theta\end{bmatrix}=\begin{bmatrix}0.7071&-0.7071\\0.7071&0.7071\end{bmatrix}$；协方差 $JPJ^\top=\begin{bmatrix}0.03927&-0.02927\\-0.02927&0.03927\end{bmatrix}$。均值偏高 $0.0238$，协方差互相关项偏离 $10\%$。

**(c) UT（$n=2,\ \kappa=1$）**：$n+\kappa=3$，$L=\sqrt{3P}$ 对角为 $(0.1732,\ 0.4534)$。5 个 sigma 点 $(r,\theta)$：

| $i$ | $r$ | $\theta$/rad | 权重 |
|---|---|---|---|
| 0 | 1.0000 | 0.7854 | $1/3$ |
| 1 | 1.1732 | 0.7854 | $1/6$ |
| 2 | 1.0000 | 1.2389 | $1/6$ |
| 3 | 0.8268 | 0.7854 | $1/6$ |
| 4 | 1.0000 | 0.3320 | $1/6$ |

分别通过 $g$，加权：均值 $(0.68329,\ 0.68329)$，协方差 $\begin{bmatrix}0.03812&-0.02585\\-0.02585&0.03812\end{bmatrix}$（Python 复核）。

### 4.3 对比

| 量 | Monte Carlo | EKF | UT ($\kappa=1$) |
|---|---|---|---|
| 均值 $x$ | 0.6834 | 0.7071 | **0.6833** |
| 方差 $\mathrm{Var}(x)$ | 0.03815 | 0.03927 | **0.03812** |
| 协方差 $\mathrm{Cov}(x,y)$ | $-0.02660$ | $-0.02927$ | $-0.02585$ |

UT 用 5 次函数求值就把均值与方差做到接近 Monte Carlo；EKF 均值偏差 $3.5\%$。当 $\kappa=0$ 时 $W_0=0$（中心点不参与），均值 $0.68315$，方差 $0.03831$，也很好；缩放 UT（$\alpha=0.5$，$\beta=2$）有 $\lambda=-1.5$，$W_0^{(m)}=-3$——**出现负权重**，协方差权重 $W_0^{(c)}=-0.25$，得到方差 $0.04019$（偏大约 5%），这是一个需要注意的现象：负权重可能导致协方差矩阵不再半正定。

## 5. UKF 算法

设模型 $x_k=f(x_{k-1},u_k)+w_k$，$z_k=h(x_k)+v_k$（加性噪声）。

**预测**：

1. 用 $(\hat x_{k-1},P_{k-1})$ 生成 sigma 点 $\mathcal X_{k-1}^{(i)}$；
2. 过 $f$：$\mathcal X_k^{(i)-}=f(\mathcal X_{k-1}^{(i)},u_k)$；
3. $\hat x_k^-=\sum W_i^{(m)}\mathcal X_k^{(i)-}$，$P_k^-=\sum W_i^{(c)}(\mathcal X_k^{(i)-}-\hat x_k^-)(\cdot)^\top+Q$。

**更新**：

1. 用 $(\hat x_k^-,P_k^-)$ 重新生成 sigma 点（或沿用预测点）；
2. 过 $h$：$\mathcal Z^{(i)}=h(\mathcal X_k^{(i)-})$；
3. $\hat z=\sum W_i^{(m)}\mathcal Z^{(i)}$，$S=\sum W_i^{(c)}(\mathcal Z^{(i)}-\hat z)(\cdot)^\top+R$，互协方差 $P_{xz}=\sum W_i^{(c)}(\mathcal X^{(i)-}-\hat x^-)(\mathcal Z^{(i)}-\hat z)^\top$；
4. $K=P_{xz}S^{-1}$，$\hat x_k=\hat x_k^-+K(z_k-\hat z)$，$P_k=P_k^--KSK^\top$。

与 EKF 的唯一区别是：用 sigma 点统计量 $P_{xz}S^{-1}$ 替代了 $P^-H^\top S^{-1}$（$H$ 是雅可比）。**如果 $h$ 是线性的，UKF 与 KF 的结果完全一致。**

计算量：每步需要 $2n+1$ 次函数求值和一次 $n\times n$ Cholesky 分解，量级与 EKF 相当，只是常数更大。

## 6. UKF 的参数与失效模式

### 6.1 参数

- $\alpha$：越小，sigma 点越靠近均值，对强非线性要谨慎（过小会使权重巨大、数值不稳）；
- $\beta$：先验高阶矩信息，高斯取 $2$；
- $\kappa$：$0$ 或 $3-n$；
- 起点推荐：$\alpha=10^{-1}$ 至 $1$，$\beta=2$，$\kappa=0$。不同文献推荐不同，这些值属于经验选择（未逐篇核对）。

### 6.2 失效与处理

| 问题 | 原因 | 对策 |
|---|---|---|
| Cholesky 失败 | $P$ 丢失正定（数值误差、负权重） | 对称化、加小对角项；用平方根 UKF |
| 角度状态均值错误 | 加权平均把 $359^\circ$ 和 $1^\circ$ 平均成 $180^\circ$ | 对角度用圆均值（对 $\sin,\cos$ 加权后 $\mathrm{atan2}$），残差用角度差并归一化 |
| 强非高斯（多峰） | 高斯假设失效 | 粒子滤波（见 [粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)） |
| 不连续函数 | sigma 点跨越不连续边界 | 平滑函数、改参数化 |
| 高维状态代价大 | $2n+1$ 次函数求值 | ESKF、稀疏方法 |

### 6.3 平方根形式与数值稳定

UKF 每一步都要对 $P$ 做 Cholesky 分解，而协方差更新 $P^+=P^--KSK^\top$ 本质是两个大矩阵相减，容易因舍入而丢失正定性。平方根 UKF 直接传播 Cholesky 因子 $S_P$（$P=S_PS_P^\top$），用 QR 分解和 Cholesky 秩一更新/降阶来完成预测与更新，因子的乘积天然半正定，条件数也只有原来的平方根。在单精度的嵌入式平台上，这几乎是必需的（参见 [数值线性代数与稀疏求解](../01_数学基础/08_数值线性代数与稀疏求解.md) 的灾难性抵消一节）。

### 6.4 调参与使用经验

1. **先在线性问题上验证**：把 $f,h$ 换成线性函数，UKF 的输出必须与 KF 逐位相同，这是最便宜的单元测试；
2. **用 Monte Carlo 做基准**：对一个典型的非线性变换，抽 $10^5$–$10^6$ 个样本，比较 EKF/UKF 的均值和协方差（正如第 4 节），可以直观看到非线性有多强，再决定是否值得换；
3. **看非线性强度而不是维度**：状态维度低、测量模型几乎线性时（例如 P5 里轮速加陀螺的平面运动），EKF 已经足够，换 UKF 只会增加计算和实现风险；
4. **观测强非线性时收益最大**：方位/距离测量、相机投影、大角度姿态运动学；
5. **噪声模型仍然是主导因素**：多数“滤波器不好用”的问题来自 $Q,R$、时间戳或模型错误，而不是线性化。换成 UKF 之前先用一致性检验（见 [估计一致性、可观性分析、NEES 与 NIS](./10_估计一致性_可观性分析_NEES与NIS.md)）排除这些原因。

## 7. 信息形式滤波

### 7.1 定义

信息矩阵 $Y=P^{-1}$，信息向量 $y=Yx$。协方差表示“不确定性”，信息表示“确定性”。

### 7.2 更新变加法

线性测量 $z=Hx+v$，$v\sim\mathcal N(0,R)$。贝叶斯更新在信息形式下为：

$$\boxed{Y^+=Y^-+H^\top R^{-1}H,\qquad y^+=y^-+H^\top R^{-1}z.}$$

**推导**：高斯先验 $\propto\exp(-\tfrac12x^\top Y^-x+y^{-\top}x)$，似然 $\propto\exp(-\tfrac12(z-Hx)^\top R^{-1}(z-Hx))$，展开乘积，二次项相加得 $Y^+$，一次项相加得 $y^+$。

**含义**：每个独立测量给出一个“信息贡献”$H^\top R^{-1}H$，直接加到信息矩阵上，顺序无关。**多传感器融合只是求和**，没有卡尔曼增益、没有矩阵求逆（直到最后要取回均值时才求一次）。

### 7.3 手算

标量先验 $x\sim\mathcal N(0,4)$，两个独立测量 $z_1=1$（$R_1=1$）、$z_2=2$（$R_2=2$），$H=1$：

- 先验：$Y^-=0.25$，$y^-=0$；
- 加测量 1：$+1/1=1$，$+1\cdot1/1=1$；加测量 2：$+1/2=0.5$，$+2/2=1$；
- $Y^+=0.25+1+0.5=1.75$，$y^+=0+1+1=2.0$；
- 取回：$x=y^+/Y^+=1.1429$，$P=1/Y^+=0.5714$。

对照顺序卡尔曼更新（Python）：第一次 $K=0.8$，$x=0.8$，$P=0.8$；第二次 $K=0.2857$，$x=1.1429$，$P=0.5714$——完全一致。

### 7.4 预测步与适用场景

信息形式的预测步较麻烦（需要对 $Y$ 取逆再回来），所以：**更新便宜、预测昂贵**，与卡尔曼形式恰好相反。适用场景：

1. **多传感器、去中心化融合**：各节点独立产生 $(H^\top R^{-1}H,\ H^\top R^{-1}z)$，相加即可，且天然地处理“无先验”（$Y=0$ 表示完全无知，卡尔曼形式要 $P=\infty$）；
2. **SLAM**：EIF（扩展信息滤波）和图优化中的 $H=J^\top\Omega J$ 就是信息矩阵，其稀疏性（相邻节点才有非零块）是高效求解的基础（见 [SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md)）；
3. **一致性/重复使用测量的风险**：信息形式让“同一信息被重复融合”的问题更清楚——重复加会使 $Y$ 虚高，过度自信。

### 7.5 当测量不独立：协方差交叉

信息形式的“相加”要求各个信息来源相互独立。现实中两个估计常常共享历史信息（例如里程计内部已经用了 IMU，又把 IMU 单独融合一次），其相关性未知。直接相加会让 $Y$ 虚高，协方差过小。保守的做法是**协方差交叉**（covariance intersection，CI）：

$$P_{CI}^{-1}=\omega P_1^{-1}+(1-\omega)P_2^{-1},\qquad P_{CI}^{-1}x_{CI}=\omega P_1^{-1}x_1+(1-\omega)P_2^{-1}x_2,\quad\omega\in[0,1].$$

不论 $x_1,x_2$ 的相关性如何，$P_{CI}$ 都不会低估真实协方差（一致）。**手算**：$x_1=1,\ P_1=1$；$x_2=2,\ P_2=2$；取 $\omega=0.5$：$P_{CI}^{-1}=0.5\times1+0.5\times0.5=0.75$，$P_{CI}=1.333$，$x_{CI}=1.333\times(0.5\times1+0.25\times2)=1.333$。若假设独立，信息直接相加，$P=1/(1+0.5)=0.667$，$x=1.333$。同一个均值，但独立假设的方差只有 CI 的一半——如果两者实际上强相关，$0.667$ 就是过度自信的数字。代价是 CI 更保守，信息被“少算”。工程上的启示：**对不确定是否独立的来源，宁可保守**，这与 P6 里多源状态融合时标注可信度的想法一致。

## 8. 卡尔曼平滑：RTS

### 8.1 滤波、预测与平滑

给定 $z_{1:N}$：

- **滤波**：$\hat x_{k\vert k}$，只用到 $k$ 的数据；
- **平滑**：$\hat x_{k\vert N}$，用全部数据，$k<N$。后面的数据对前面的状态也有信息，因为状态之间由动力学相连。

平滑估计的协方差不大于滤波协方差：$P_{k\vert N}\preceq P_{k\vert k}$。

### 8.2 RTS 平滑器

先做一遍正向卡尔曼滤波，存下每步的 $\hat x_{k\vert k},P_{k\vert k},\hat x_{k+1\vert k},P_{k+1\vert k}$。然后从 $k=N-1$ 向 $0$ 后向递推：

$$C_k=P_{k\vert k}F^\top P_{k+1\vert k}^{-1},$$
$$\hat x_{k\vert N}=\hat x_{k\vert k}+C_k\big(\hat x_{k+1\vert N}-\hat x_{k+1\vert k}\big),$$
$$P_{k\vert N}=P_{k\vert k}+C_k\big(P_{k+1\vert N}-P_{k+1\vert k}\big)C_k^\top.$$

**推导思路**：条件在 $z_{1:k}$ 下，$(x_k,x_{k+1})$ 是联合高斯，互协方差 $P_{k\vert k}F^\top$。给定 $x_{k+1}$ 的条件期望为 $\hat x_{k\vert k}+C_k(x_{k+1}-\hat x_{k+1\vert k})$，并且给定 $x_{k+1}$ 后 $x_k$ 与未来测量条件独立（马尔可夫性质），所以只需要把 $x_{k+1}$ 换成它的平滑分布（均值 $\hat x_{k+1\vert N}$、协方差 $P_{k+1\vert N}$）再边缘化即可，得到上式。

**直觉**：$C_k$ 是“未来对过去的影响系数”。如果下一步的预测很确定（$P_{k+1\vert k}$ 小），那么后来的信息（$\hat x_{k+1\vert N}-\hat x_{k+1\vert k}$）更多地传回 $k$。

### 8.3 手算：标量随机游走

模型 $x_k=x_{k-1}+w$，$q=0.1$；测量 $z_k=x_k+v$，$r=0.5$；初值 $\hat x_0=0$，$P_0=1$。测量 $z=(0.9,\ 1.1,\ 0.8,\ 1.2)$（示例，非项目实测）。

**正向滤波**（Python 复核）：

| $k$ | $\hat x_{k\vert k-1}$ | $P_{k\vert k-1}$ | 增益 $K$ | $\hat x_{k\vert k}$ | $P_{k\vert k}$ |
|---|---|---|---|---|---|
| 1 | 0 | 1.1 | 0.6875 | 0.6188 | 0.3438 |
| 2 | 0.6188 | 0.4438 | 0.4702 | 0.8450 | 0.2351 |
| 3 | 0.8450 | 0.3351 | 0.4013 | 0.8270 | 0.2006 |
| 4 | 0.8270 | 0.3006 | 0.3755 | 0.9670 | 0.1877 |

**后向平滑**（$F=1$）：

- $k=3$：$C_3=P_{3\vert3}/P_{4\vert3}=0.2006/0.3006=0.6674$；$\hat x_{3\vert4}=0.8270+0.6674\times(0.9670-0.8270)=0.9204$；$P_{3\vert4}=0.2006+0.6674^2\times(0.1877-0.3006)=0.1504$；
- $k=2$：$C_2=0.7016$，$\hat x_{2\vert4}=0.8979$，$P_{2\vert4}=0.1442$；
- $k=1$：$C_1=0.7746$，$\hat x_{1\vert4}=0.8350$，$P_{1\vert4}=0.1640$。

对比：第 1 步滤波估计 $0.6188$（方差 $0.3438$），平滑后 $0.8350$（方差 $0.1640$）。滤波时第一步只看到一个测量 $0.9$ 且先验均值 0，被拉向先验；平滑后“知道后面的测量都在 $0.8$–$1.2$”，估计向上修正，方差下降约一半。最后一步 $k=4$ 平滑等于滤波（没有未来数据）。

### 8.3b 与批量最小二乘（信息矩阵）的验证

线性高斯情形下，RTS 平滑应与“一次性解整条链的最大后验问题”给出同样的结果。把 $x_0,\dots,x_4$ 作为未知量，先验 $x_0\sim\mathcal N(0,1)$、4 个动力学约束（权重 $1/q$）、4 个测量（权重 $1/r$），构成 $5\times5$ 的信息矩阵 $\Lambda$ 与信息向量 $\eta$，解 $\Lambda m=\eta$ 并取 $\Lambda^{-1}$ 的对角线。Python 结果：均值 $(0.7591,\ 0.8350,\ 0.8979,\ 0.9204,\ 0.9670)$，方差 $(0.2264,\ 0.1640,\ 0.1442,\ 0.1504,\ 0.1877)$，其中 $k=1,\dots,4$ 的均值与方差逐位等于上表的 RTS 结果（$0.8350/0.1640$、$0.8979/0.1442$、$0.9204/0.1504$、$0.9670/0.1877$）。矩阵 $\Lambda$ 是三对角的（链结构），所以批量求解也是 $O(N)$，这与 [SLAM问题_前端后端_位姿图优化](../07_SLAM与建图/01_SLAM问题_前端后端_位姿图优化.md) 中“位姿图是稀疏最小二乘”的结论是同一个事实：**滤波是对这个大问题的前向消元，平滑是回代**。这个等价性让你在面试里可以把“卡尔曼平滑”和“因子图优化”统一成一句话：线性情形下它们是同一个问题的两种求解顺序，非线性情形下则通过迭代线性化（高斯牛顿）推广。

### 8.4 应用

1. **离线轨迹后处理**：事后用全部数据平滑，得到比实时滤波更平滑、更准确的轨迹（例如作为评测真值、训练数据标注、地图构建的位姿初值）；
2. **参数估计 EM 算法**：E 步需要平滑；
3. **图优化的联系**：线性高斯情形下，RTS 平滑等价于对整条链做一次最大后验估计（批量最小二乘），不同的是它利用了链结构，复杂度是 $O(N)$。非线性时对应迭代的图优化。

## 9. 固定滞后平滑

### 9.1 想法

实时系统等不到“全部数据”，但可以等“几步”：用 $z_{1:k+L}$ 估计 $x_k$（滞后 $L$ 步）。输出比滤波精确，代价是 $L\Delta t$ 的延迟。

### 9.2 数值：标量稳态

同一个模型（$q=0.1$，$r=0.5$），把一条长序列中间位置 $k=40$ 的后验方差，随滞后 $L$ 变化（Python，用信息矩阵求逆得到精确值）：

| 滞后 $L$ | 0（滤波） | 1 | 2 | 3 | 5 | 10 | 30 |
|---|---|---|---|---|---|---|---|
| 后验方差 | 0.1791 | 0.1379 | 0.1210 | 0.1140 | 0.1099 | 0.1091 | 0.1091 |

读法：滞后 1 步方差降 $23\%$，滞后 3 步降 $36\%$，滞后 5 步已经接近极限 $0.1091$（降 $39\%$）；再往后增益可忽略。**收益递减**：通常只需要几步滞后，因为远处的测量与当前状态的相关性被过程噪声“冲淡”了。过程噪声 $q$ 越小（动力学越确定），可用的有效滞后越长。

### 9.3 实现方式

1. 对状态扩维，把最近 $L+1$ 个状态堆叠（增广状态 $[x_k;x_{k-1};\dots;x_{k-L}]$），用普通卡尔曼滤波；维度 $(L+1)n$；
2. 用 RTS 递推限定在窗口内（滑动窗口），每步重新平滑最近 $L$ 步；
3. 在优化框架里做滑动窗口 + 边缘化（见 [IMU预积分与因子图估计](./08_IMU预积分与因子图估计.md)）。

## 10. 在本项目里怎么用

> 说明：CV 的 P5 使用 EKF/AMCL，P1/P3/P7 使用互补滤波/姿态估计。UKF、信息滤波和平滑属于延伸知识，不是 CV 中的实现；面试里讲成“我了解 EKF 之外的选择以及何时需要它们”。

- **P5 EKF 的局限追问**：问“EKF 在什么情况下会不好用”，答强非线性（大角度不确定性、方位测量）、需要雅可比、均值偏差；列出 UKF 和粒子滤波作为替代，并说明为什么项目里选 EKF（模型接近线性、计算量低、`robot_localization` 现成，见 [卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[P5 EKF 融合定位](../../01_项目课程/P5_车式移动机器人自主导航/04_EKF融合定位.md)）；
- **P5 偏航角**：状态含角度时，无论 EKF 还是 UKF 都要处理 $\pm\pi$ 环绕，UKF 的均值需要圆均值；
- **P5 里程计与传感器时间延迟**：延迟的测量（如激光匹配结果晚到）可用固定滞后平滑或回滚重放处理（见 [里程计与传感器融合](./04_里程计与传感器融合.md)、[时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)）；
- **离线分析**：对实机日志里的轨迹做 RTS 平滑，可以得到更干净的“参考轨迹”用于评估在线滤波的精度；
- **P6 多源融合**：多设备状态融合时，信息形式的“加法”结构是去中心化融合的数学基础，同时提醒不要重复使用同一信息（见 [机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)）。

## 11. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| UKF 输出角度跳变 | 对角度直接加权平均 | 用圆均值，残差归一化 |
| UKF 报告协方差非正定 | 数值误差、负权重 | 对称化+抖动项；改 $\alpha$、$\kappa$；平方根形式 |
| UKF 与 EKF 在线性问题上结果不同 | 实现错误 | 线性情形二者应完全一致，用来做单元测试 |
| 用了 UKF 仍然不一致 | 噪声模型、时间戳或模型本身有问题 | UKF 只改善线性化误差，不能修复模型 |
| 信息形式更新后无法直接读出估计 | 忘记取 $x=Y^{-1}y$ | 需要时才解 $Yx=y$（用分解，不显式求逆） |
| 同一测量被融合两次，协方差过小 | 信息被重复计数 | 追踪信息来源，用协方差交叉（CI）等保守融合 |
| RTS 平滑结果出现不连续 | 存的先验/后验协方差错位 | 严格对应 $k\vert k$ 与 $k+1\vert k$ |
| 在线系统使用了全局平滑 | 违背因果性 | 改用固定滞后或窗口 |
| 平滑后轨迹“过平滑”，细节丢失 | $Q$ 偏小 | 重新整定 $Q$（平滑对 $Q$ 更敏感） |

## 12. 面试怎么问、怎么答

**Q1. What is the difference between an EKF and a UKF?**

> The EKF linearises the nonlinear functions with a first-order Taylor expansion and needs Jacobians. The UKF avoids linearisation: it picks a small set of sigma points that match the mean and covariance, passes each one through the true nonlinear function, and recomputes the mean and covariance from the weighted outputs. It captures the mean and covariance to at least second order, needs no Jacobians, and has a similar cost, about 2n+1 function evaluations. In a polar-to-Cartesian example with a 15-degree bearing noise, the EKF mean was off by about 3.5 percent, while the UKF was within 0.01 percent of Monte Carlo.

中文：EKF 对非线性函数一阶泰勒线性化、需要雅可比；UKF 选一小组匹配均值协方差的 sigma 点，各自通过真实非线性函数，再加权恢复均值协方差；均值协方差至少二阶精度、不需雅可比、代价相近（$2n+1$ 次求值）；极坐标例子里 EKF 均值偏差约 3.5%，UKF 与 Monte Carlo 差不到 0.01%。

**Q2. When would you use the information form of the Kalman filter?**

> When I fuse many independent measurements or want to handle a completely unknown prior, because the update is just adding the information matrix and vector, and the order does not matter. It is also the natural form for decentralised fusion and for SLAM, where the information matrix is sparse. The price is that the prediction step is more expensive, so it suits problems with frequent updates and rare predictions.

中文：融合大量独立测量或先验完全未知时，因为更新就是信息矩阵/向量相加且与顺序无关；是去中心化融合和 SLAM（信息矩阵稀疏）的自然形式；代价是预测步更贵，适合更新频繁、预测少的问题。

**Q3. What is a smoother and how does it differ from a filter?**

> A filter estimates the state at time k using measurements up to k. A smoother uses measurements from the future as well, so it is more accurate, but it is not causal. The RTS smoother runs a forward Kalman filter, stores the predictions and posteriors, and then goes backward with a gain C that pulls the earlier estimates toward the later ones. I use it offline for trajectory post-processing, and a fixed-lag version online when I can afford a small delay.

中文：滤波用到 $k$ 的测量估计 $k$ 时刻状态；平滑还用未来的测量，更准但非因果；RTS 先正向滤波并存储预测和后验，再用增益 $C$ 后向把早期估计拉向后期估计；离线做轨迹后处理，在线能容忍小延迟时用固定滞后版本。

**Q4. Why can the smoothed variance never be larger than the filtered one, and by how much does it help?**

> Because smoothing conditions on more data, and extra data cannot increase the posterior variance of a Gaussian. In my scalar example, the first state's variance dropped from 0.34 to 0.16. For a fixed-lag smoother the benefit saturates quickly: with the same model, a lag of one step gave about 23 percent lower variance, three steps about 36 percent, and beyond five steps almost nothing more.

中文：平滑在更多数据下取条件，多出的数据不会增大高斯后验方差；标量例子里第一个状态方差由 0.34 降到 0.16；固定滞后收益很快饱和：同一模型滞后 1 步方差降约 23%，3 步约 36%，5 步后几乎不再改善。

**Q5. What can go wrong with a UKF?**

> The sigma point generation needs a Cholesky factor, so a covariance that loses positive definiteness breaks it. Angles need circular means, otherwise averaging 359 and 1 degrees gives 180. The scaled version can give negative weights, which may make the covariance indefinite. And the UKF still assumes a unimodal Gaussian, so it cannot fix a multimodal posterior or a wrong noise model; for those I would look at particle filters or fix the model.

中文：sigma 点需要 Cholesky，协方差失去正定会失败；角度需圆均值，否则 359° 和 1° 平均成 180°；缩放版本可能负权重使协方差不定；UKF 仍假设单峰高斯，修复不了多峰后验或错误噪声模型，需要粒子滤波或修正模型。

## 13. 本章要点速记

1. EKF 一阶线性化导致均值/协方差偏差并需要雅可比；示例中 $x$ 均值偏高 3.5%。
2. UT：$2n+1$ 个 sigma 点，$\mu\pm\sqrt{(n+\kappa)P}$，$W_0=\kappa/(n+\kappa)$，$W_i=1/(2(n+\kappa))$。
3. 对线性函数精确；非线性时均值协方差至少二阶；缩放 UT 注意负权重。
4. UKF：$P_{xz}S^{-1}$ 取代 $P^-H^\top S^{-1}$；线性时与 KF 完全一致。
5. 角度状态用圆均值；Cholesky 失败要对称化/平方根形式。
6. 信息形式：$Y^+=Y^-+H^\top R^{-1}H$，$y^+=y^-+H^\top R^{-1}z$，融合 = 加法；预测昂贵。
7. 示例：先验 $\mathcal N(0,4)$ 融合 $z=1,2$ 得 $x=1.1429$，$P=0.5714$，与顺序更新一致。
8. RTS：$C_k=P_{k\vert k}F^\top P_{k+1\vert k}^{-1}$；$\hat x_{k\vert N}=\hat x_{k\vert k}+C_k(\hat x_{k+1\vert N}-\hat x_{k+1\vert k})$。
9. 平滑方差 $\le$ 滤波方差；标量例第 1 步 $0.344\to0.164$。
10. 固定滞后：收益快速饱和（1 步降 23%，3 步 36%，5 步 39%）。

## 14. 自测题

1. $n=3$，$\kappa=0$ 的 UT 有几个 sigma 点，各权重？（7 个；$W_0=0$，其余 $1/6$。）
2. 标量 $x\sim\mathcal N(0,1)$，$y=x^2$，真实 $E[y]$ 与一阶线性化给出的值？（真实 1；线性化 $g(0)=0$。）
3. 信息形式下，三个独立测量各给出 $H^\top R^{-1}H=2$，先验 $Y^-=1$，后验方差？（$Y^+=7$，$P=1/7$。）
4. RTS 的最后一步 $k=N$ 平滑值与滤波值的关系？（相等，没有未来数据。）
5. 为什么平滑对 $Q$ 更敏感？（平滑要通过动力学把信息传回过去 $C_k$ 依赖 $Q$。）
6. 固定滞后 5 步、采样周期 20 ms，引入的输出延迟？（100 ms。）

## 15. 相关链接

- 上一章：[时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)；下一章：[IMU预积分与因子图估计](./08_IMU预积分与因子图估计.md)
- 相关基础：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)、[粒子滤波与 AMCL](./05_粒子滤波与AMCL.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)
- 项目：[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P6](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 模块说明：[06_状态估计 README](./README.md)
