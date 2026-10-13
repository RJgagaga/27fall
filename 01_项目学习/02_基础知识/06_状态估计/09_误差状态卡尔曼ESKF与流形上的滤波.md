# 误差状态卡尔曼滤波（ESKF）与流形上的滤波

> **一句话定位**：姿态不是向量，四元数有 4 个分量却只有 3 个自由度，直接把它塞进 EKF 会让协方差奇异、线性化失真。误差状态卡尔曼滤波（Error-State Kalman Filter，ESKF）的做法是：**让一个“名义状态”（nominal state）按 IMU 大步向前积分，让滤波器只估计“名义值与真值之间的小误差”**。误差始终很小、在 $\mathbb{R}^3$ 里、接近零均值——线性化几乎无损，协方差满秩。估计出来的误差再“注入”回名义状态，并清零。这是现代 IMU 姿态估计、惯性导航和视觉/激光惯性里程计里最常见的滤波器。

**本章学完你能：**

- 说清为什么直接对四元数做 EKF 有问题，以及 ESKF 如何用“名义状态 + 误差状态”解决；
- 区分真值、名义值、误差，写出 $R=\hat R\,\mathrm{Exp}(\delta\theta)$ 的右（局部）误差定义，并推导 $\dot{\delta\theta}=-[\hat\omega]_\times\delta\theta-\delta b_g-n_g$；
- 写出 15 维误差状态 $[\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g]$ 的连续时间动力学矩阵 $F$、噪声驱动矩阵与离散 $Q_d$ 的量级；
- 完整说出 ESKF 的预测—更新—注入—重置四个步骤，并手算一次“重力方向”姿态更新（含 $H=[R^\top\hat z]_\times$ 的秩 2 与偏航不可观）；
- 解释重置雅可比 $G=I-[\tfrac12\delta\theta]_\times$ 及其大小，用 Python 数值验证；
- 用一个一维“位置—速度—加速度计偏置”例子，看到 ESKF 如何用 GPS 位置把偏置估到 $0.1$ 附近；
- 比较 ESKF 与 EKF、与流形上的滤波（左/右不变误差）的关系；
- 用英文回答 "Why error state / what is injected and reset / why is yaw unobservable from gravity" 等问题。

**前置知识**：[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)（预测/更新、雅可比）、[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)（$\mathrm{Exp}$、右扰动、$[\cdot]_\times$）、[欧拉角、四元数、姿态表示](../03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md)、[传感器：IMU、编码器、激光雷达](./01_传感器_IMU_编码器_激光雷达.md)、[互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)。

## 目录

1. 问题：为什么不直接对四元数做 EKF
2. 符号与约定
3. 名义状态、误差状态与真值
4. 误差状态的运动学推导
5. 15 维误差状态的动力学矩阵
6. 噪声与离散化
7. 更新、注入与重置
8. 手算一：重力方向修正姿态
9. 手算二：一维加速度计偏置估计
10. ESKF 与 EKF、流形上滤波的比较
11. 在本项目里怎么用
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 问题：为什么不直接对四元数做 EKF

IMU 姿态估计的朴素 EKF：状态 $x=[q;\ b_g]$（四元数 4 维 + 偏置 3 维），预测用 $\dot q=\tfrac12q\otimes\omega$，更新用加速度计。这样做有三个麻烦。

**其一，协方差奇异。** 单位四元数满足 $\lVert q\rVert=1$，真实自由度只有 3。EKF 对四元数的 4×4 协方差块要么奇异（rank 3），要么被噪声撑成满秩却没有物理意义。奇异的 $P$ 在做 Cholesky、求逆（信息形式）时会崩溃。

**其二，更新不守约束。** EKF 的更新 $x\leftarrow x+K\nu$ 是向量加法，更新后的四元数 $q+\delta q$ 模长不再是 1。常规补救是更新后归一化，但这种“事后补丁”没有反映在协方差里。

**其三，线性化不准。** 全局姿态状态可以很大（旋转几十度、几百度），姿态运动学对四元数是双线性的，雅可比随姿态变化剧烈，线性化误差大。

ESKF 的思路是**换一个状态来滤波**：不滤波姿态本身，而滤波“姿态误差”。姿态误差是一个小旋转向量 $\delta\theta\in\mathbb{R}^3$，无约束、满秩、值很小，线性化几乎精确。

## 2. 符号与约定

| 符号 | 含义 | 单位 / 维度 |
|---|---|---|
| $x_t$ | 真值状态（true state） | |
| $\hat x$ | 名义状态（nominal state），由 IMU 积分得到 | |
| $\delta x$ | 误差状态（error state） | 15 维 |
| $R$、$\hat R$ | 真实、名义姿态（机体到世界） | |
| $\delta\theta$ | 姿态误差（旋转向量，局部/右扰动） | rad |
| $\delta p,\delta v$ | 位置、速度误差（世界系） | m，m/s |
| $\delta b_a,\delta b_g$ | 加速度计、陀螺偏置误差 | m/s²，rad/s |
| $\omega_m,a_m$ | IMU 测量 | rad/s，m/s² |
| $\hat\omega=\omega_m-\hat b_g$，$\hat a=a_m-\hat b_a$ | 名义的去偏置读数 | |
| $n_g,n_a,n_{bg},n_{ba}$ | 陀螺噪声、加速度计噪声、两个偏置的随机游走驱动噪声 | |
| $g$ | 重力向量，$(0,0,-9.81)$ | m/s² |
| $F,\Phi,Q_d$ | 误差动力学矩阵、离散状态转移、离散过程噪声 | |
| $H,K,P$ | 观测雅可比、增益、误差协方差 | |
| $G$ | 重置雅可比 | |

**约定**：采用 Hamilton 四元数（与 [欧拉角、四元数、姿态表示](../03_运动学与坐标变换/02_欧拉角_四元数_姿态表示.md) 一致），右（局部）误差 $R=\hat R\,\mathrm{Exp}(\delta\theta)$。同类文献也有使用左（全局）误差 $R=\mathrm{Exp}(\delta\theta_g)\hat R$ 的写法，两者相差 $\delta\theta_g=\hat R\,\delta\theta$；协方差、雅可比必须与所选定义一致（见李群章第 11 节）。Solà 的教程《Quaternion kinematics for the error-state Kalman filter》是该滤波器的常用参考，此处引用的是其思路，具体章节编号与公式未逐条核对。

## 3. 名义状态、误差状态与真值

真值 = 名义值 ⊕ 误差：

$$p=\hat p+\delta p,\quad v=\hat v+\delta v,\quad R=\hat R\,\mathrm{Exp}(\delta\theta),\quad b_a=\hat b_a+\delta b_a,\quad b_g=\hat b_g+\delta b_g.$$

**滤波器的分工**：

1. 名义状态 $\hat x$：用 IMU 读数按非线性运动学高速积分（几百到上千 Hz），不含噪声，会随时间漂移；
2. 误差状态 $\delta x$：由线性（线性化）误差动力学传播，用 $P$ 描述其不确定性；每当有外部测量（GPS、视觉、激光、重力方向）时，用卡尔曼更新估计 $\delta x$；
3. **注入（injection）**：$\hat x\leftarrow\hat x\oplus\delta\hat x$，把估计出的误差加回名义状态；
4. **重置（reset）**：$\delta\hat x\leftarrow0$，协方差做相应变换。

**为什么好**：（1）$\delta\theta$ 是 3 维、无约束，协方差 $3\times3$ 满秩；（2）误差小，线性化近似对 $\delta\theta\ll1$ 几乎精确；（3）误差动力学变化缓慢，可以低于 IMU 频率更新协方差，节省计算；（4）名义状态里的非线性（四元数积分）由精确的非线性运算完成，不被线性化污染。

## 4. 误差状态的运动学推导

### 4.1 姿态误差

真实角速度 $\omega=\omega_m-b_g-n_g=\hat\omega-\delta b_g-n_g$（因为 $b_g=\hat b_g+\delta b_g$）。真实与名义姿态的微分方程：

$$\dot R=R\,[\omega]_\times,\qquad \dot{\hat R}=\hat R\,[\hat\omega]_\times.$$

对 $R=\hat R\,\mathrm{Exp}(\delta\theta)$ 求导：

$$\dot R=\dot{\hat R}\,\mathrm{Exp}(\delta\theta)+\hat R\,\frac d{dt}\mathrm{Exp}(\delta\theta)=\hat R[\hat\omega]_\times\mathrm{Exp}(\delta\theta)+\hat R\,\mathrm{Exp}(\delta\theta)\,[J_r(\delta\theta)\,\dot{\delta\theta}]_\times.$$

另一方面 $\dot R=R[\omega]_\times=\hat R\,\mathrm{Exp}(\delta\theta)[\omega]_\times$。两边左乘 $\hat R^\top$、右乘 $\mathrm{Exp}(\delta\theta)^\top$，并利用 $\mathrm{Exp}(\delta\theta)[\omega]_\times\mathrm{Exp}(-\delta\theta)=[\mathrm{Exp}(\delta\theta)\omega]_\times$（伴随性质）整理：

$$[\omega]_\times=\mathrm{Exp}(-\delta\theta)[\hat\omega]_\times\mathrm{Exp}(\delta\theta)+[J_r\dot{\delta\theta}]_\times.$$

即 $J_r\,\dot{\delta\theta}=\omega-\mathrm{Exp}(-\delta\theta)\hat\omega$。一阶近似（小量）：$J_r\approx I$，$\mathrm{Exp}(-\delta\theta)\hat\omega\approx\hat\omega-\delta\theta\times\hat\omega$：

$$\dot{\delta\theta}\approx\omega-\hat\omega+\delta\theta\times\hat\omega=-\delta b_g-n_g-[\hat\omega]_\times\delta\theta.$$

$$\boxed{\dot{\delta\theta}=-[\hat\omega]_\times\,\delta\theta-\delta b_g-n_g}.$$

### 4.2 速度误差

真实：$\dot v=R(a_m-b_a-n_a)+g$。名义：$\dot{\hat v}=\hat R(a_m-\hat b_a)+g$。代入 $R\approx\hat R(I+[\delta\theta]_\times)$，$b_a=\hat b_a+\delta b_a$，取一阶：

$$\dot{\delta v}=\hat R(I+[\delta\theta]_\times)(\hat a-\delta b_a-n_a)-\hat R\hat a\approx\hat R[\delta\theta]_\times\hat a-\hat R\,\delta b_a-\hat R\,n_a.$$

用 $[\delta\theta]_\times\hat a=-[\hat a]_\times\delta\theta$：

$$\boxed{\dot{\delta v}=-\hat R[\hat a]_\times\delta\theta-\hat R\,\delta b_a-\hat R\,n_a}.$$

### 4.3 位置与偏置误差

$$\dot{\delta p}=\delta v,\qquad \dot{\delta b_a}=n_{ba},\qquad \dot{\delta b_g}=n_{bg}.$$

（偏置误差由随机游走驱动；重力 $g$ 被认为已知，若同时估计重力，则增加 $\delta g$ 状态。）

## 5. 15 维误差状态的动力学矩阵

误差状态 $\delta x=[\delta p;\delta v;\delta\theta;\delta b_a;\delta b_g]$，$\dot{\delta x}=F\delta x+Bn$：

$$F=\begin{bmatrix}0&I&0&0&0\\0&0&-\hat R[\hat a]_\times&-\hat R&0\\0&0&-[\hat\omega]_\times&0&-I\\0&0&0&0&0\\0&0&0&0&0\end{bmatrix}.$$

噪声向量 $n=[n_a;n_g;n_{ba};n_{bg}]$ 通过 $B$ 进入速度、姿态和两个偏置的行：速度行 $-\hat R$（对 $n_a$），姿态行 $-I$（对 $n_g$），偏置行 $I$。

**离散化**：采样周期 $\Delta t$ 很小时，$\Phi=\exp(F\Delta t)\approx I+F\Delta t$（或用二阶展开）。姿态块 $\Phi_{\theta\theta}=\mathrm{Exp}(-\hat\omega\Delta t)$ 是精确的。**数值例子**：$\hat\omega=(0,0,0.5)$ rad/s，$\Delta t=0.01$ s，$\Phi_{\theta\theta}=R_z(-0.005)$，即 $\begin{bmatrix}0.999988&0.005&0\\-0.005&0.999988&0\\0&0&1\end{bmatrix}$（Python 复核）：误差向量绕 $z$ 转了 $-0.005$ rad，和名义姿态转 $+0.005$ rad 方向相反（因为误差是在机体系下表达的）。

## 6. 噪声与离散化

### 6.1 离散噪声协方差

连续噪声密度（见 [随机信号…Allan 方差](../04_信号与系统/04_随机信号_自相关_功率谱_白噪声与Allan方差.md)）：陀螺 ARW $N_g$，加速度计 VRW $N_a$，偏置随机游走 $K_g,K_a$。一个采样周期 $\Delta t$ 内累积的离散过程噪声：

$$Q_d\approx\mathrm{diag}\big(\underbrace{0}_{\delta p},\ N_a^2\Delta t\,I,\ N_g^2\Delta t\,I,\ K_a^2\Delta t\,I,\ K_g^2\Delta t\,I\big).$$

位置块几乎没有直接噪声（它的不确定性来自速度的积分）。**数值例子**：$N_g=2\times10^{-4}$ rad/√s，$\Delta t=0.01$ s：姿态块 $4\times10^{-10}$ rad² per step（Python 复核），约 100 步（1 s）后累积到 $4\times10^{-8}$ rad²，即 $2\times10^{-4}$ rad 标准差，与 ARW 一致。

### 6.2 协方差预测

$$P\leftarrow\Phi P\Phi^\top+Q_d.$$

因为误差动力学是线性的，预测不依赖于误差的数值，只依赖于名义状态（通过 $\hat R,\hat a,\hat\omega$ 出现在 $F$ 中）。

## 7. 更新、注入与重置

### 7.1 更新

测量 $z=h(x_t)+v$，$v\sim\mathcal N(0,R_m)$（避免与旋转矩阵 $R$ 混淆，这里把测量噪声协方差记为 $R_m$）。用链式法则得到对误差状态的雅可比：

$$H=\frac{\partial h}{\partial\delta x}=\frac{\partial h}{\partial x_t}\frac{\partial x_t}{\partial\delta x},$$

其中 $\dfrac{\partial x_t}{\partial\delta x}$ 把“真值对误差的导数”写出来：位置、速度、偏置块是单位阵，姿态块是 $\partial(R)/\partial\delta\theta$。常见观测的 $H$：

| 观测 | $h(x)$ | $H$ 的非零块 |
|---|---|---|
| GPS/位置 | $p$ | $\partial h/\partial\delta p=I$ |
| 速度 | $v$ | $\partial h/\partial\delta v=I$ |
| 重力方向（单位向量，机体系） | $R^\top\hat z$ | $\partial h/\partial\delta\theta=[\hat R^\top\hat z]_\times$ |
| 磁力计方向 | $R^\top m_w$ | $[\hat R^\top m_w]_\times$ |
| 视觉位姿 | $(p,R)$ | $I$ 与 $I$（相对 $\delta p,\delta\theta$），或经 $\mathrm{Log}$ 残差 |

标准卡尔曼更新：

$$K=PH^\top(HPH^\top+R_m)^{-1},\quad \delta\hat x=K\,(z-h(\hat x)),\quad P\leftarrow(I-KH)P\ \ \text{（或 Joseph 形式）}.$$

### 7.2 注入

$$\hat p\leftarrow\hat p+\delta\hat p,\ \ \hat v\leftarrow\hat v+\delta\hat v,\ \ \hat b\leftarrow\hat b+\delta\hat b,\ \ \hat R\leftarrow\hat R\,\mathrm{Exp}(\delta\hat\theta)\ \ (\text{或}\ \hat q\leftarrow\hat q\otimes q\{\delta\hat\theta\}).$$

注入姿态是**乘法**，用指数映射，保证 $\hat R$ 始终是合法旋转，这是 ESKF 避免归一化补丁的关键。

### 7.3 重置

注入之后，误差状态的参考点变了，需要把 $\delta\hat x$ 置零，并让协方差跟着变：

$$\delta\hat x\leftarrow0,\qquad P\leftarrow GPG^\top,\qquad G=\mathrm{diag}\big(I,\ I,\ I-[\tfrac12\delta\hat\theta]_\times,\ I,\ I\big).$$

$G$ 来自旧误差定义与新误差定义之间的雅可比（旋转的一阶变换）。当 $\delta\hat\theta$ 很小时，$G\approx I$，很多实现直接省略重置；但在大修正或精度要求高时应该保留。

### 7.4 整个流程

```text
循环:
  1. 读 IMU:  名义状态 x_hat 非线性积分 (R,p,v)
  2. 协方差预测:  P <- Phi P Phi^T + Qd
  3. 有测量时:
        y = z - h(x_hat);  H = d h / d(dx);  K = P H^T (H P H^T + Rm)^-1
        dx_hat = K y;      P <- (I - K H) P (Joseph)
        注入: x_hat <- x_hat (+) dx_hat  (姿态乘法)
        重置: dx_hat = 0;  P <- G P G^T
```

### 7.5 多速率、门控与初始化

实际系统里传感器速率差别很大：IMU 100–1000 Hz，GPS 1–10 Hz，视觉/激光 10–30 Hz，磁力计 50 Hz。ESKF 的流程天然支持多速率：名义状态每个 IMU 周期更新；协方差预测可以与 IMU 同频，也可以在两次测量之间累积若干步 $\Phi$ 再一次性更新（误差动力学慢）；每个测量到达时单独做一次更新。要点有三条。

第一，**时间对齐**：测量到达时的时间戳与当前名义状态不一定对应，晚到的测量要么回滚到对应时刻再重放（保存最近一段名义状态与协方差的历史），要么把延迟作为状态估计，要么对名义状态做插值（见 [时间同步、坐标系一致性与数据有效性](./06_时间同步_坐标系一致性与数据有效性.md)）。

第二，**门控**：对每个测量先算马氏距离 $\nu^\top S^{-1}\nu$，超过 $\chi^2$ 阈值就拒绝，防止一次坏测量（GPS 多径、视觉误匹配、加速度计在剧烈机动时偏离重力）把状态拉飞。对加速度计重力更新，常再加一个条件：测得的加速度模长与 $9.81$ 的偏差小于阈值时才使用，或者按偏差放大 $R_m$。

第三，**初始化**：初始名义姿态可以由静止时的重力方向（横滚、俯仰）和磁力计（偏航）给出；初始 $P$ 要诚实地反映这些估计的不确定性（例如偏航没有参考时给一个大值），偏置初值取静止窗口内陀螺的平均读数，对应的 $P$ 取该平均的方差。初始化太乐观（$P$ 过小）会让滤波器长时间不相信后续测量，是常见的“收敛很慢”的原因。

## 8. 手算一：重力方向修正姿态

### 8.1 设定

名义姿态 $\hat R=I$（水平）；真实姿态相对名义有小倾斜 $\delta\theta_{\text{true}}=(0.02,\ -0.01,\ 0)$ rad（绕 $x$ 转 $1.15^\circ$，绕 $y$ 转 $-0.57^\circ$；示例，非项目实测）。加速度计静止，测得的“向上”单位向量在机体系中为 $m=R^\top\hat z=\mathrm{Exp}(-\delta\theta)\hat z\approx\hat z-\delta\theta\times\hat z=(0.01,\ 0.02,\ 1)$（Python 复核：$(0.009999,\ 0.019998,\ 0.99975)$）。预测测量 $h(\hat x)=\hat R^\top\hat z=(0,0,1)$。新息

$$y=m-h(\hat x)=(0.010,\ 0.020,\ -0.00025).$$

先验协方差 $P=0.05^2I$（$\sigma=0.05$ rad $=2.9^\circ$），测量噪声 $R_m=0.02^2I$。

### 8.2 雅可比与增益

$H=[\hat R^\top\hat z]_\times=[\hat z]_\times=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&0\end{bmatrix}$。**秩为 2**：第三行全零，说明测量对 $\delta\theta_z$（偏航误差）完全不敏感——**重力方向无法观测偏航**（绕竖直轴转动不改变“向上”这个向量）。

$S=HPH^\top+R_m=\mathrm{diag}(0.0029,\ 0.0029,\ 0.0004)$，$K=PH^\top S^{-1}$，非零块为 $\pm\dfrac{0.0025}{0.0029}=\pm0.8621$：

$$K=\begin{bmatrix}0&0.8621&0\\-0.8621&0&0\\0&0&0\end{bmatrix}.$$

### 8.3 结果

$$\delta\hat\theta=Ky=(0.01724,\ -0.00862,\ 0).$$

与真值 $(0.02,\ -0.01,\ 0)$ 相比，估计出了其 $86\%$（其余 $14\%$ 是因为先验有一定权重，不完全信任测量）。注入：$\hat R\leftarrow\hat R\,\mathrm{Exp}(\delta\hat\theta)$，修正后与真实姿态的差 $\lVert\hat R-R\rVert_F=0.0044$。

**协方差**：横滚、俯仰的标准差由 $0.05$ 降到 $0.0186$ rad；偏航的标准差**仍是 $0.05$**（Python 复核：$P^+$ 对角 $(3.45\times10^{-4},\ 3.45\times10^{-4},\ 2.5\times10^{-3})$）。滤波器"诚实地"表示：它对偏航一无所知。要观测偏航需要磁力计、视觉、GNSS 航向等。

**重置**：$G=I-[\tfrac12\delta\hat\theta]_\times$，$\lVert G-I\rVert_{\max}=0.0086$；重置后 $P$ 的变化最大 $1.9\times10^{-5}$（相对 $P^+$ 的 5%）。可见小修正时重置影响很小。

### 8.4 大角度与迭代

若初始倾斜很大，如 $\delta\theta=(0.3,\ -0.2,\ 0)$ rad，先验 $0.5$ rad：一次更新估得 $(0.2931,\ -0.1954,\ 0)$，线性化误差约 $2$–$3\%$。注入后以新的名义姿态重新线性化，再更新一次，剩余误差只有 $\sim0.003$ rad（Python）。这种“更新—注入—重线性化”的迭代就是迭代 EKF 的思想，ESKF 因为每次注入后误差归零，天然适合。

## 9. 手算二：一维加速度计偏置估计

### 9.1 设定

一维运动：真加速度 $a(t)=0.5\sin(0.5t)$ m/s²；加速度计有未知偏置 $b=0.1$ m/s²，噪声 $\sigma_a=0.05$ m/s²；每秒一次 GPS 位置，$\sigma=0.5$ m。IMU 100 Hz。误差状态 $[\delta p,\delta v,\delta b]$（示例，非项目实测）：

$$F_d=\begin{bmatrix}1&\Delta t&-\tfrac12\Delta t^2\\0&1&-\Delta t\\0&0&1\end{bmatrix},\quad H=[1\ 0\ 0].$$

名义状态用 $\hat a=a_m-\hat b$ 积分，初值 $\hat b=0$，$P_0=\mathrm{diag}(1,1,0.2^2)$。

### 9.2 结果

Python 仿真 60 s，偏置估计与 $1\sigma$：

| $t$/s | $\hat b$ | 估计标准差 |
|---|---|---|
| 10 | 0.0909 | 0.0420 |
| 20 | 0.1129 | 0.0149 |
| 30 | 0.1038 | 0.0108 |
| 40 | 0.0973 | 0.0089 |
| 50 | 0.0981 | 0.0078 |
| 60 | 0.1011 | 0.0070 |

真值为 $0.1$。估计在 10 s 时已经落在真值 $0.2\sigma$ 之内，60 s 时偏差仅 $0.001$。**注意：这依赖于加速度的激励**（$a(t)$ 变化）。若运动是匀速（$a=0$），偏置与位置的可区分性很差：偏置产生的位置漂移 $\tfrac12bt^2$ 与初始速度误差、位置误差无法完全分开（可观性问题，见下一章）。

### 9.3 怎样解读这张表

表中偏置标准差从 $0.042$ 降到 $0.007$，大约按时间的 $-1/2$ 次方下降，这是线性高斯估计"测量越多越准"的典型形状：每秒一次 GPS，每次提供固定量的信息，累积的信息与测量次数成正比，标准差与其平方根成反比。同时，偏置的估计误差（$\hat b-b$）并不单调：20 s 时 $+0.013$，40 s 时 $-0.003$，波动的幅度大致与同一时刻的 $1\sigma$ 相当——这恰恰是"一致"的表现。如果在多次独立运行里反复出现误差远大于 $\sigma$，就说明噪声参数或模型有问题，这时要用下一章的 NEES/NIS 检验。另一个值得观察的现象是位置误差 $\hat p-p$ 保持在约 $0.2$–$0.5$ m，比单次 GPS 的 $0.5$ m 噪声略好；速度误差保持在 $0.04$–$0.16$ m/s。偏置估计准了，名义状态的漂移才被抑制，这是"估计偏置能改善导航精度"的直接证据。

## 10. ESKF 与 EKF、流形上滤波的比较

| 对比 | 直接 EKF（四元数） | ESKF | 流形上滤波（不变 EKF 等） |
|---|---|---|---|
| 滤波的状态 | 全状态（含四元数） | 误差状态 | 群上的元素，误差用群运算定义 |
| 协方差维数 | 16（四元数 4 维，奇异） | 15（满秩） | 与自由度一致 |
| 约束保持 | 需事后归一化 | 注入用 $\mathrm{Exp}$，天然保持 | 天然保持 |
| 线性化点 | 全状态，误差大 | 误差接近 0，精确 | 视误差定义，可使误差动力学独立于轨迹 |
| 实现 | 简单，易错 | 中等，推导繁但模块化 | 需要群论基础 |
| 一致性/可观性 | 常出现虚假可观 | 同 EKF，需 FEJ 等措施 | 部分情形性质更好（文献） |

**流形上滤波的统一视角**：ESKF 可以看作“流形上的 EKF”：状态是流形上的点，用 $\boxplus$（$R\boxplus\delta\theta=R\,\mathrm{Exp}(\delta\theta)$）把切空间向量加到流形点上，用 $\boxminus$ 表示两个流形点之差。许多实现（例如若干激光惯性里程计）把滤波器写成对任意流形通用的形式。不变 EKF（invariant EKF，Barrau 与 Bonnabel 的工作，年份与细节未核对）选用对群作用不变的误差定义（左不变或右不变），使误差动力学在某些问题上只依赖测量而不依赖轨迹估计，从而得到更好的收敛与一致性性质。选哪种误差定义取决于传感器：机体系测量（陀螺、相机）常用右/局部误差，世界系测量（GPS）常用左/全局误差；选择的是方便性，而不改变估计问题本身。

## 11. 在本项目里怎么用

> 说明：CV 的姿态估计是 Mahony 互补滤波 + EKF/AMCL 平面定位，没有写 ESKF。ESKF 是 IMU 姿态/惯导领域的标准结构，本章用来回答“互补滤波、EKF、ESKF 有什么关系”。

- **P1 / P3 / P7 姿态估计**：Mahony 互补滤波用 $e=\hat a\times a_{\text{meas}}$（估计的重力方向与测得重力方向的叉积）作为修正量，这个叉积正是本章 §8 的新息 $y=m-h(\hat x)$ 乘以 $[\hat z]_\times$ 的结果（在小角度下 $[\hat z]_\times^\top y\propto\delta\theta$）。互补滤波可以看成一个**增益固定的 ESKF**：不跟踪协方差，用常数 $k_P,k_I$ 代替卡尔曼增益，积分项 $k_I$ 估计陀螺偏置（对应误差状态 $\delta b_g$）。ESKF 的优势是增益随不确定性自动调节，并且能同时融合多个传感器（见 [互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)、[IMU/AHRS 与姿态估计 (P1)](../../01_项目课程/P1_AIRS球形机器人平衡控制/03_IMU_AHRS与姿态估计.md)、[IMU/AHRS 与云台姿态 (P3)](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/05_IMU_AHRS与云台姿态.md)）。
- **偏航不可观**：P1/P3 的 IMU 姿态没有磁力计时，偏航会缓慢漂移，这是 §8 的"重力不观测偏航"的实际后果；云台应用里用视觉目标作为相对参考（见 P3 云台章节）。
- **P5 EKF**：平面里程计的状态只有 $(x,y,\psi)$，偏航是一维 $SO(2)$，不需要三维姿态的 ESKF 机制，但"误差用 $\boxminus$ 定义，角度差要归一化"的思想相同（见 [卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)）。
- **P2 VR 姿态**：手柄姿态四元数的相对运算与 §7.2 的注入操作同构（乘法而非加法）。

## 12. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 姿态更新后四元数模长偏离 1 | 用了加法注入 | 注入用 $\hat q\otimes q\{\delta\theta\}$，并周期性归一化 |
| 滤波器在大修正后协方差不一致 | 忽略重置雅可比 $G$ | 实现 $G$，或迭代更新 |
| 偏航误差无限增长，但协方差不变小 | 偏航本来就不可观 | 加磁力计/视觉/GNSS 航向，或接受漂移 |
| 左右误差定义混用，雅可比符号错 | $\delta\theta_g=\hat R\delta\theta$ 未区分 | 全文统一，数值差分检验 |
| 加速度计更新时机器人在加速，姿态被拉歪 | 假设“静止只测重力”不成立 | 根据加速度模长门控，增大 $R_m$，动态时降低权重 |
| 偏置估计发散 | 不可观（缺激励）或 $Q$ 过小 | 检查激励，增大偏置过程噪声 |
| 协方差变成非对称/非正定 | 数值误差 | Joseph 形式 + 对称化 |
| 名义状态漂移太大，线性化失效 | 更新太稀疏，误差累积大 | 提高更新频率，限制注入幅度 |
| 把测量噪声 $R$ 与旋转矩阵 $R$ 混淆 | 符号冲突 | 测量噪声写 $R_m$，旋转写 $\hat R$ |

## 13. 面试怎么问、怎么答

**Q1. Why do we use an error-state filter instead of filtering the orientation directly?**

> Orientation lives on a manifold. A quaternion has four numbers for three degrees of freedom, so a direct EKF covariance is singular or meaningless, and the additive update breaks the unit norm. In the error-state filter I integrate a nominal state with the full nonlinear kinematics, and the filter estimates only the small error, a three-dimensional rotation vector with a full-rank covariance. The error is small so linearisation is accurate, and I inject it back with a multiplication, so the orientation stays valid.

中文：姿态在流形上；四元数 4 个数 3 个自由度，直接 EKF 协方差奇异或无意义，加法更新破坏单位模；误差状态滤波用完整非线性运动学积分名义状态，滤波器只估计小误差（3 维旋转向量、满秩协方差）；误差小线性化精确，注入用乘法，姿态保持合法。

**Q2. Explain injection and reset.**

> After a measurement update the filter estimate of the error is added back into the nominal state: positions, velocities and biases additively, orientation by composing with the exponential of the error rotation. That moves the reference point of the error, so the error estimate is set to zero. The covariance is also transformed by the Jacobian of the reset, which for the orientation block is the identity minus half the skew-symmetric matrix of the error. For small corrections this is almost the identity, but I keep it when corrections are large.

中文：更新后把误差估计加回名义状态：位置/速度/偏置用加法，姿态用误差旋转的指数映射复合；这改变了误差的参考点，所以把误差估计置零；协方差用重置雅可比变换，姿态块是 $I-\tfrac12[\delta\theta]_\times$；小修正时近似单位阵，大修正时必须保留。

**Q3. Why can't gravity tell you the yaw angle?**

> The accelerometer at rest measures the gravity direction in the body frame. Rotating about the vertical axis leaves that direction unchanged, so the measurement Jacobian with respect to the yaw error is zero, and the Jacobian matrix has rank two. In my small example the update reduced roll and pitch uncertainty from 0.05 to 0.019 radians but left the yaw uncertainty at 0.05. To observe yaw you need another reference such as a magnetometer, a GNSS heading or vision.

中文：静止时加速度计测量机体系重力方向，绕竖直轴转动不改变该方向，所以对偏航误差的雅可比为零，$H$ 秩为 2；示例中更新使横滚俯仰不确定性由 0.05 降到 0.019 rad，偏航仍是 0.05；观测偏航需要磁力计、GNSS 航向或视觉。

**Q4. How is a complementary filter related to an ESKF?**

> A Mahony complementary filter integrates the gyro and corrects the orientation with the cross product between the estimated and measured gravity directions, plus an integral term that estimates the gyro bias. That is the same structure as the error-state filter: the cross product is the innovation projected onto the orientation error, and the integral state is the bias error. The difference is that the complementary filter uses fixed gains, while the ESKF computes the gain from the covariance, so it adapts to the uncertainty and can fuse several sensors consistently.

中文：Mahony 互补滤波积分陀螺，并用估计与测得重力方向的叉积修正，再加积分项估计陀螺偏置；这与误差状态滤波结构相同：叉积是新息在姿态误差上的投影，积分状态是偏置误差；区别是互补滤波增益固定，ESKF 由协方差算增益，随不确定性自适应并能一致地融合多传感器。

**Q5. What are the typical sources of inconsistency in an ESKF and how do you detect them?**

> The usual ones are linearisation errors when the nominal state is far from the truth, unobservable directions like yaw or absolute position getting spuriously reduced in uncertainty, wrong noise parameters, and time-stamp errors. I detect them with the normalised innovation squared against its chi-square bounds, innovation whiteness tests, and in simulation the normalised estimation error squared over many runs. Remedies include iterating the update, first-estimate Jacobians, inflating noise, and fixing the time alignment.

中文：常见来源：名义状态远离真值导致线性化误差、不可观方向（偏航、绝对位置）不确定性被虚假减小、噪声参数错误、时间戳错误；用 NIS 对照卡方界、新息白性检验、仿真中多次运行的 NEES 来检测；补救：迭代更新、FEJ、放大噪声、修正时间对齐。

## 14. 本章要点速记

1. ESKF：名义状态（非线性积分）+ 误差状态（线性高斯滤波）；更新后注入并重置。
2. 右误差：$R=\hat R\,\mathrm{Exp}(\delta\theta)$；$\dot{\delta\theta}=-[\hat\omega]_\times\delta\theta-\delta b_g-n_g$。
3. $\dot{\delta v}=-\hat R[\hat a]_\times\delta\theta-\hat R\delta b_a-\hat R n_a$，$\dot{\delta p}=\delta v$，偏置误差为随机游走。
4. 15 维误差状态 $[\delta p,\delta v,\delta\theta,\delta b_a,\delta b_g]$，协方差满秩。
5. 离散：$\Phi\approx I+F\Delta t$，$Q_d$ 对角：$N_a^2\Delta t,\ N_g^2\Delta t,\ K_a^2\Delta t,\ K_g^2\Delta t$。
6. 注入姿态用乘法：$\hat R\leftarrow\hat R\,\mathrm{Exp}(\delta\hat\theta)$；重置 $G=I-[\tfrac12\delta\hat\theta]_\times$。
7. 重力观测：$H=[\hat R^\top\hat z]_\times$，秩 2，偏航不可观；示例横滚俯仰 $\sigma$ 0.05→0.0186，偏航仍 0.05。
8. 示例：新息 $(0.010,0.020,-0.00025)$ → $\delta\hat\theta=(0.01724,-0.00862,0)$，恢复真值的 86%。
9. 一维偏置估计：60 s 后 $\hat b=0.1011$（真值 0.1），$\sigma=0.007$；依赖加速度激励。
10. 互补滤波 = 固定增益的 ESKF；流形滤波的左/右误差选择要与传感器及全文一致。

## 15. 自测题

1. 为什么 ESKF 的协方差是 15×15 而不是 16×16？（姿态误差用 3 维旋转向量，不是 4 维四元数。）
2. 机体系角速度 $\hat\omega=(0,0,1)$ rad/s，$\Delta t=0.02$ s，$\Phi_{\theta\theta}$ 是绕 $z$ 转多少？（$-0.02$ rad。）
3. 静止水平放置，磁力计缺失，哪些姿态误差可通过重力修正？（横滚、俯仰；偏航不可。）
4. 重力观测中新息为 $y=(0.02,0,-0.0002)$，$K$ 的非零块为 $\pm0.8$，求 $\delta\hat\theta$。（$\delta\hat\theta=Ky=(0,\ -0.8\times0.02,\ 0)=(0,-0.016,0)$。）
5. 为什么注入完成后要把误差状态清零？（误差是相对名义状态定义的，名义状态已吸收了估计，新的误差均值应为 0。）
6. 互补滤波的 $k_I$ 对应 ESKF 中的哪个量？（陀螺偏置误差 $\delta b_g$ 的估计。）

## 16. 相关链接

- 上一章：[IMU预积分与因子图估计](./08_IMU预积分与因子图估计.md)；下一章：[估计一致性、可观性分析、NEES 与 NIS](./10_估计一致性_可观性分析_NEES与NIS.md)
- 相关基础：[李群李代数与流形优化入门](../01_数学基础/06_李群李代数与流形优化入门.md)、[互补滤波与 Mahony AHRS](./02_互补滤波与Mahony_AHRS.md)、[卡尔曼滤波与 EKF](./03_卡尔曼滤波与EKF.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[06_状态估计 README](./README.md)
