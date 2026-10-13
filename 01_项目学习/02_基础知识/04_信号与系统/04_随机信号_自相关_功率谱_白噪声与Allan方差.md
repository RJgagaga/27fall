# 随机信号：自相关、功率谱、白噪声与 Allan 方差

> **一句话定位**：传感器噪声不是"一个标准差"能说清的——白噪声、慢漂移、随机游走、有色噪声在时间上的"记忆"不同，对估计的影响也完全不同。本章给随机信号一套语言（自相关与功率谱），解释白噪声的离散与连续表示，并用 Allan 方差把 IMU 数据手册上的 ARW、零偏不稳定性、速率随机游走翻译成卡尔曼滤波的 $Q$、$R$。

**本章学完你能：**

- 定义平稳随机过程的均值、自相关 $R_{xx}(\tau)$，并写出 Wiener–Khinchin 关系 $S_{xx}(\omega)=\int R_{xx}(\tau)e^{-j\omega\tau}d\tau$；
- 区分连续白噪声（强度/谱密度）与离散白噪声（方差），推出 $\sigma_d^2=q/\Delta t$ 并解释"为什么采样越快、每个采样的噪声越大"；
- 推导一阶高斯—马尔可夫过程的自相关与谱，手算并仿真验证离散 AR(1) 的 $\phi=e^{-\Delta t/T}$；
- 解释随机游走（积分白噪声）的方差随时间线性增长，并区分角度随机游走（ARW）与零偏随机游走；
- 手算 Allan 方差的定义与 ARW、速率随机游走的理论曲线，用仿真数据从曲线上读出 $N$、$K$；
- 把数据手册的噪声参数换算为滤波器的 $Q$ 与 $R$，并说出该换算的假设与限制；
- 用样本自相关做"白性检验"，判断滤波残差是否合格；
- 用英文回答 "What is Allan variance / how do you get Q and R from a datasheet / what is a random walk" 等问题。

**前置知识**：[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)（期望、方差、协方差）、[信号与系统基础](./01_信号与系统基础_卷积_傅里叶_拉普拉斯_Z变换.md)（傅里叶变换、LTI）、[频谱分析：FFT、窗函数、PSD 与振动诊断](./03_频谱分析_FFT_窗函数_PSD与振动诊断.md)（PSD 与单位）、[传感器：IMU、编码器、激光雷达](../06_状态估计/01_传感器_IMU_编码器_激光雷达.md)（IMU 误差模型）。

## 目录

1. 问题：一个"标准差"为什么不够
2. 符号与单位
3. 随机过程、平稳性与自相关
4. 白噪声：连续与离散
5. 有色噪声：高斯—马尔可夫过程
6. 随机游走与积分
7. 样本自相关与白性检验
8. Allan 方差
9. 从 Allan 参数到滤波器的 $Q$ 与 $R$
10. 统计置信度与注意事项
11. 在本项目里怎么用
12. 常见错误与故障现象
13. 面试怎么问、怎么答
14. 本章要点速记
15. 自测题
16. 相关链接

---

## 1. 问题：一个"标准差"为什么不够

你拿到一块 IMU，把它静置 10 分钟，陀螺输出的标准差是 $0.002$ rad/s。是不是可以把 $R=0.002^2$ 填进卡尔曼滤波，万事大吉？

答案通常是**不行**，原因是这个标准差把几种时间尺度完全不同的误差混在了一起：

1. **快速的白噪声**：每个采样独立，相互之间没有记忆。滤波器对它的处理最简单，平均 $n$ 次，标准差降低 $\sqrt n$ 倍；
2. **缓慢的零偏漂移**：时间相关，相邻采样几乎一样。平均 $n$ 次不会降低它，而且它积分之后以 $t$ 或 $t^{3/2}$ 增长，才是姿态长期漂移的主因；
3. **随机游走**：漂移本身又是白噪声的积分，方差随时间线性增长。

只给出"总体标准差"，你既不知道该平滑多少、也不知道偏置应以多快速度变化。**谱（或自相关）告诉你每个时间尺度上有多少噪声**，Allan 方差则专为这类长期慢变误差设计，一张图就能把多种噪声分开。

本章的主线是：自相关 → 功率谱 → 白噪声的连续/离散 → 有色与随机游走 → Allan 方差 → $Q$、$R$。

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $x(t)$、$x_k$ | 连续、离散随机信号 | 视信号（如 rad/s） |
| $\mu=E[x]$ | 均值 | 同 $x$ |
| $R_{xx}(\tau)=E[(x(t)-\mu)(x(t+\tau)-\mu)]$ | 自协方差/自相关（去均值） | $x^2$ |
| $r[k]=R_{xx}[k]/R_{xx}[0]$ | 归一化自相关序列（自相关系数） | 无量纲 |
| $S_{xx}(\omega)$ | 双边功率谱密度 | $x^2/(\text{rad/s})$ |
| $q$ | 连续白噪声强度，$E[n(t)n(t+\tau)]=q\,\delta(\tau)$ | $x^2\cdot$s |
| $\sigma_d^2$ | 离散白噪声方差 | $x^2$ |
| $N$ | 角度随机游走系数（ARW） | rad/√s（或 °/√h） |
| $K$ | 速率随机游走系数（RRW） | rad/s/√s（陀螺），即 rad·s$^{-3/2}$ |
| $B$ | 零偏不稳定性 | rad/s（或 °/h） |
| $\tau$ | 平均时间（Allan 方差的横轴） | s |
| $\sigma_A(\tau)$ | Allan 偏差 | 同 $x$ |

## 3. 随机过程、平稳性与自相关

### 3.1 随机过程

随机过程是"随时间变化的随机变量"：每个时刻 $t$ 都是一个随机变量，每次实验得到一条样本轨迹。传感器静置记录就是一条样本轨迹。

### 3.2 宽平稳

若均值与时刻无关、自相关只与时间差 $\tau$ 有关，则称宽平稳（wide-sense stationary）。大多数传感器噪声在恒温静置时可近似看作平稳。对平稳过程，在较弱的条件（遍历性）下，**时间平均等于集合平均**，所以我们用一条足够长的记录就能估计自相关与功率谱。

### 3.3 自相关的含义

$R_{xx}(\tau)$ 衡量"现在的值"与"$\tau$ 秒后的值"的线性相关程度。$R_{xx}(0)=\sigma^2$ 是方差。记忆越长，$R_{xx}(\tau)$ 衰减越慢。归一化后 $r\in[-1,1]$。

### 3.4 Wiener–Khinchin 定理

自相关与功率谱是一对傅里叶变换：

$$S_{xx}(\omega)=\int_{-\infty}^{\infty}R_{xx}(\tau)e^{-j\omega\tau}d\tau,\qquad R_{xx}(\tau)=\frac1{2\pi}\int_{-\infty}^{\infty}S_{xx}(\omega)e^{j\omega\tau}d\omega.$$

令 $\tau=0$：$\sigma^2=R_{xx}(0)=\frac{1}{2\pi}\int S_{xx}(\omega)d\omega$，方差等于谱下的面积（与上一章"PSD 积分得方差"一致）。**自相关衰减得快 ⇔ 谱宽；自相关衰减得慢 ⇔ 谱窄（集中在低频）。**

## 4. 白噪声：连续与离散

### 4.1 理想白噪声

连续白噪声 $n(t)$：自相关为冲激 $R_{nn}(\tau)=q\,\delta(\tau)$，谱为常数 $S_{nn}(\omega)=q$（双边）。任意两个不同时刻不相关。它的方差是 $R(0)=q\delta(0)=\infty$，物理上不存在，只是对"带宽远大于系统带宽"的噪声的理想化。

### 4.2 采样后的离散白噪声

传感器（或 ADC）在采样周期 $\Delta t$ 内对信号取平均或积分，则采样值的方差是

$$\boxed{\sigma_d^2=\frac{q}{\Delta t}}.$$

**推导**：采样值 $x_k=\frac1{\Delta t}\int_{k\Delta t}^{(k+1)\Delta t}n(t)dt$。方差 $E[x_k^2]=\frac1{\Delta t^2}\iint E[n(t)n(s)]dtds=\frac1{\Delta t^2}\iint q\,\delta(t-s)dtds=\frac{q\Delta t}{\Delta t^2}=\frac q{\Delta t}$。

**含义**：采样越快（$\Delta t$ 越小），每个采样的方差越大——因为窗口更短，平均掉的噪声更少。但单位时间内的"信息量"（谱密度 $q$）不变。把 $n$ 个采样平均，等价于时间窗口放长 $n$ 倍，方差降为 $\sigma_d^2/n=q/(n\Delta t)$，只取决于总时长 $T=n\Delta t$：$q/T$。

### 4.3 数据手册里的噪声密度

陀螺的"角度随机游走"$N$（rad/√s）是白噪声角速度的强度的平方根：$N=\sqrt q$。于是

$$\sigma_{d}=\frac{N}{\sqrt{\Delta t}}.$$

**手算**（示例，非项目实测）：$N=2\times10^{-4}$ rad/√s（等价于 $0.01146^\circ/\sqrt{\text{s}}=0.6875^\circ/\sqrt{\text{h}}$），采样 $\Delta t=0.01$ s：$\sigma_d=2\times10^{-4}/0.1=2\times10^{-3}$ rad/s；换成 $\Delta t=0.005$ s（200 Hz）则 $\sigma_d=2.83\times10^{-3}$ rad/s。**同一个传感器，采样率翻倍，每个采样的标准差大了 $\sqrt2$ 倍**，而角度积分后的累积不确定度 $N\sqrt t$ 不变：10 s 后为 $6.3\times10^{-4}$ rad（$0.036^\circ$），600 s 后为 $4.9\times10^{-3}$ rad（$0.28^\circ$）。

> **注意**：如果在滤波器里不加修正地把 100 Hz 下测的标准差用于 200 Hz，会把噪声低估。换采样率必须重新用 $q$ 换算。

### 4.4 带宽与单边/双边

数据手册常给的是**单边**噪声密度 $a$（ASD，单位 /√Hz，例如 °/s/√Hz）。单边 PSD 是 $a^2$，在带宽 $B$ 内的方差是 $a^2B$；采样率 $f_s$ 下可用带宽是 $f_s/2$，所以 $\sigma_d^2=a^2f_s/2=a^2/(2\Delta t)$。对照 $\sigma_d^2=q/\Delta t$ 可得 $q=a^2/2$，即 $N=a/\sqrt2$。**手算**：若手册给出 $a=2.83\times10^{-4}$ rad/s/√Hz，则 $N=2.83\times10^{-4}/\sqrt2=2.0\times10^{-4}$ rad/√s，与前面的示例一致。换算时务必看清是 Hz 还是 rad/s、单边还是双边，常见的因子 2、$\sqrt2$ 和 $2\pi$ 错误会让 $R$ 偏出数倍。

### 4.5 量化噪声：编码器测速为什么"毛"

数字传感器的输出是量化的：步长 $\Delta$ 的均匀量化误差在 $[-\Delta/2,\Delta/2]$ 上近似均匀分布，方差 $\Delta^2/12$。编码器的位置读数有量化，用相邻两次位置做差分求速度，误差是两个独立量化误差之差，方差 $2\cdot\Delta^2/12=\Delta^2/6$，所以速度噪声标准差

$$\sigma_v=\frac{\Delta}{\sqrt6\,\Delta t}.$$

**手算**（示例，非项目实测）：每转 4096 个计数的编码器，$\Delta=2\pi/4096=1.534\times10^{-3}$ rad，$\Delta t=5$ ms：$\sigma_v=1.534\times10^{-3}/(2.449\times0.005)=0.125$ rad/s。Python 仿真匀速 1 rad/s 的量化差分，样本标准差 $0.134$ rad/s，与理论同量级（匀速时量化误差有确定性的周期模式，不严格独立，所以略有偏差）。采样周期越短，差分测速的噪声越大（正比于 $1/\Delta t$），这与 4.2 节的结论同向，也是为什么高采样率下要用 M/T 法或滤波测速（见 [数字滤波器](./02_数字滤波器_FIR_IIR_采样与混叠.md) 的 dirty derivative）。

## 5. 有色噪声：高斯—马尔可夫过程

### 5.1 一阶高斯—马尔可夫

许多慢变误差（温度漂移、低频零偏起伏）可以用一阶高斯—马尔可夫过程描述：

$$\dot x=-\frac1Tx+w,\qquad E[w(t)w(s)]=q_w\delta(t-s).$$

稳态方差 $\sigma^2=q_wT/2$，自相关

$$R_{xx}(\tau)=\sigma^2e^{-\lvert\tau\rvert/T},\qquad S_{xx}(\omega)=\frac{2\sigma^2T}{1+(\omega T)^2}.$$

即**白噪声通过一阶低通**。相关时间 $T$：自相关降到 $e^{-1}=0.368$ 的时间；谱的转折频率 $f_c=1/(2\pi T)$。

### 5.2 离散化：AR(1)

用精确离散化，$x_{k+1}=\phi x_k+w_k$，$\phi=e^{-\Delta t/T}$，$\mathrm{Var}(w_k)=\sigma^2(1-\phi^2)$（保持稳态方差为 $\sigma^2$）。自相关 $r[k]=\phi^{\lvert k\rvert}$。

**手算**：$T=1$ s，$\Delta t=0.1$ s，$\sigma=1$：$\phi=e^{-0.1}=0.9048$；$q_k=1\times(1-0.8187)=0.1813$；理论 $r[1]=0.9048$，$r[2]=0.8187$，$r[10]=\phi^{10}=e^{-1}=0.3679$。

**仿真验证**（20 万点，Python）：样本方差 $0.9926$，样本 $r[1]=0.9043$，$r[2]=0.8178$，$r[3]=0.7401$（理论 $0.9048,0.8187,0.7408$），$r[10]=0.3667$。转折频率 $f_c=1/(2\pi\cdot1)=0.159$ Hz。

### 5.3 对估计器的含义

卡尔曼滤波的基本假设是过程噪声和测量噪声都是白的。有色噪声必须**扩维**：把 $x$ 作为状态加入滤波器（如零偏作为状态），再由白噪声驱动。这就是 IMU 零偏要作为状态来估计的理由（见 [卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md) 与 [误差状态卡尔曼 ESKF](../06_状态估计/09_误差状态卡尔曼ESKF与流形上的滤波.md)）。如果把有色噪声直接当作白噪声测量噪声塞进 $R$，滤波器会过于自信：它以为相邻观测独立，实际上相邻观测的误差同向，平均并不能消除。

## 6. 随机游走与积分

### 6.1 定义

白噪声的积分叫维纳过程/随机游走：$x(t)=\int_0^tn(s)ds$。方差

$$\mathrm{Var}[x(t)]=qt,\qquad \sigma_x(t)=\sqrt{q\,t}.$$

标准差随 $\sqrt t$ 增长，**没有稳态**。谱 $S\propto1/\omega^2$。

### 6.2 对陀螺的两种后果

1. **角度随机游走（ARW）**：角速度的白噪声被积分成角度 → $\sigma_\theta=N\sqrt t$；
2. **零偏随机游走（速率随机游走，RRW）**：零偏本身 $\dot b=K\,\eta(t)$（$\eta$ 为单位白噪声），$b(t)$ 的标准差 $K\sqrt t$；该零偏再被积分到角度，角度误差的标准差 $\dfrac{Kt^{3/2}}{\sqrt3}$。

**手算**（示例）：$K=10^{-5}$ rad/s/√s，$t=600$ s：零偏漂移标准差 $K\sqrt t=2.45\times10^{-4}$ rad/s（$0.014^\circ$/s）；积分后的角度误差 $Kt^{3/2}/\sqrt3=0.0849$ rad $=4.86^\circ$。对比同一 $t$ 的 ARW 角度误差 $0.28^\circ$：**长时间尺度上零偏随机游走远比白噪声危险**。10 s 时：ARW $0.036^\circ$，RRW $0.0105^\circ$——短时间内白噪声占主导，长时间反过来。两者在某个时间尺度相交，那正是 Allan 曲线最低点附近。

## 7. 样本自相关与白性检验

### 7.1 估计

给定 $N$ 个样本，样本自相关系数

$$\hat r[k]=\frac{\sum_{n=0}^{N-k-1}(x_n-\bar x)(x_{n+k}-\bar x)}{\sum_{n=0}^{N-1}(x_n-\bar x)^2}.$$

### 7.2 白性检验

对白噪声，$\hat r[k]$（$k\ne0$）近似服从 $\mathcal N(0,1/N)$。因此约 95% 的值应落在 $\pm2/\sqrt N$ 之内。$N=1000$ 时界限 $\pm0.0632$。

**数值验证**：生成 1000 个白噪声，前 20 个滞后系数的最大绝对值 $0.0623$，没有一个越界（Python）。

### 7.3 用在哪里：检验滤波器

如果卡尔曼滤波器的模型匹配，它的**新息（innovation，测量残差）** 应为零均值白噪声，且方差等于预测的 $S_k$。观察新息序列的自相关：若某一滞后显著越界，说明模型漏掉了某种动态（有色噪声没建模、时间延迟、偏置未估计）。这是 [估计一致性、可观性分析、NEES 与 NIS](../06_状态估计/10_估计一致性_可观性分析_NEES与NIS.md) 里的一致性检验的重要补充。

## 8. Allan 方差

### 8.1 定义

给定采样间隔 $\Delta t$ 的静置数据 $y_k$（例如陀螺角速度），取平均时间 $\tau=m\Delta t$。把数据分成长度为 $m$ 的连续簇，第 $j$ 簇的平均 $\bar y_j(\tau)$。Allan 方差定义为相邻簇平均之差的半均方：

$$\sigma_A^2(\tau)=\frac12\Big\langle\big(\bar y_{j+1}(\tau)-\bar y_j(\tau)\big)^2\Big\rangle.$$

Allan 偏差（Allan deviation）是其平方根 $\sigma_A(\tau)$。直觉：看"相邻两段平均值差多少"。平均时间短时，主要是白噪声的影响；平均时间长时，零偏漂移的影响变大。

### 8.2 几种典型噪声的理论形态

| 噪声类型 | $\sigma_A^2(\tau)$ | 双对数图斜率（$\sigma_A$） | 读取方法 |
|---|---|---|---|
| 角度随机游走（速率白噪声，ARW） | $N^2/\tau$ | $-1/2$ | $\tau=1$ s 处 $\sigma_A=N$ |
| 零偏不稳定性（闪烁噪声） | $\approx\big(\frac{2\ln2}{\pi}\big)B^2\ \Rightarrow\ \sigma_A\approx0.664B$ | 0（平底） | 平底最小值 $/0.664$ |
| 速率随机游走（RRW） | $K^2\tau/3$ | $+1/2$ | $\tau=3$ s 处 $\sigma_A=K$ |
| 速率斜坡（线性漂移） | $R^2\tau^2/2$ | $+1$ | |

零偏不稳定性的系数 $0.664$ 来自标准（IEEE Std 952 一类陀螺测试规范）中的约定，此处未逐条核对，使用时请对照你所用的数据手册的定义。

**白噪声项的推导**：速率白噪声 $y_k$ 方差 $\sigma_d^2=N^2/\Delta t$。簇平均 $\bar y_j$ 的方差 $\sigma_d^2/m$，相邻两簇平均独立，故差的方差 $2\sigma_d^2/m$，半均方得 $\sigma_A^2=\sigma_d^2/m=\dfrac{N^2}{\Delta t\,m}=\dfrac{N^2}{\tau}$。✓

**RRW 项的推导思路**：零偏为随机游走时，相邻簇平均之差正比于 $K\sqrt\tau$，具体系数 $1/\sqrt3$ 来自相邻簇平均的相关性（簇内的积分），完整推导可对 $\bar y_{j+1}-\bar y_j$ 的方差做二重积分得到，结论 $\sigma_A^2=K^2\tau/3$。

### 8.3 仿真与读数

构造仿真陀螺：$N=2\times10^{-4}$ rad/√s，$K=10^{-5}$ rad/s/√s，$\Delta t=0.01$ s，$2\times10^6$ 点（20000 s）。每步白噪声标准差 $N/\sqrt{\Delta t}=2\times10^{-3}$，零偏每步增量 $K\sqrt{\Delta t}=10^{-6}$。理论曲线 $\sigma_A^2=N^2/\tau+K^2\tau/3$。重叠 Allan 偏差计算结果（Python）：

| $\tau$/s | 仿真 $\sigma_A$ | 理论 $\sigma_A$ |
|---|---|---|
| 0.01 | $2.00\times10^{-3}$ | $2.00\times10^{-3}$ |
| 0.1 | $6.32\times10^{-4}$ | $6.32\times10^{-4}$ |
| 1 | $2.01\times10^{-4}$ | $2.00\times10^{-4}$ |
| 10 | $6.56\times10^{-5}$ | $6.58\times10^{-5}$ |
| 30 | $4.88\times10^{-5}$ | $4.83\times10^{-5}$ |
| 100 | $6.55\times10^{-5}$ | $6.11\times10^{-5}$ |
| 300 | $1.19\times10^{-4}$ | $1.01\times10^{-4}$ |
| 1000 | $2.27\times10^{-4}$ | $1.83\times10^{-4}$ |

读数：（1）$\tau=1$ s 处 $\sigma_A=2.01\times10^{-4}$ → $N\approx2.0\times10^{-4}$ ✓；（2）曲线最小点在 $\tau\approx30$ s，值 $4.9\times10^{-5}$，理论最小点 $\tau^\ast=\sqrt3N/K=34.6$ s、$\sigma_{\min}=\sqrt{2NK/\sqrt3}=4.81\times10^{-5}$；（3）长 $\tau$ 端斜率 $+1/2$，用末端点估计 $K\approx\sigma_A(\tau)\sqrt{3/\tau}$ 得 $1.24\times10^{-5}$，比真值 $10^{-5}$ 高 24%。

### 8.4 为什么 $K$ 估偏了：统计置信度

长 $\tau$ 处簇数很少：$\tau=1000$ s 只有 $20000/1000=20$ 个簇。Allan 偏差估计的相对不确定度近似 $1/\sqrt{2(M-1)}$（$M$ 为簇数）：$M=20$ 时约 $16\%$，对随机游走这样非平稳的过程，单次实现的偏差还会更大。结论：**读 $K$ 要用足够长的数据（至少 $\tau_{\max}$ 的 10 倍以上，越长越好），且不要只信曲线最右端的点**。同时，数据手册给的 $B$、$N$、$K$ 是典型值，具体到你这块芯片、这个温度可能差很多。

## 9. 从 Allan 参数到滤波器的 $Q$ 与 $R$

### 9.1 对应关系

以陀螺为例，滤波器离散时间步长 $\Delta t_k$（注意：是滤波器更新周期，不一定是传感器采样周期）：

| 物理量 | 滤波器里的量 | 公式 | 示例值（$N=2\times10^{-4}$，$K=10^{-5}$，$\Delta t_k=0.005$ s） |
|---|---|---|---|
| 角速度白噪声（ARW） | 陀螺测量噪声方差（作为过程噪声进入姿态预测） | $\sigma_g^2=N^2/\Delta t_k$ | $8\times10^{-6}$ (rad/s)² |
| 零偏随机游走 | 零偏状态的过程噪声方差 | $Q_b=K^2\Delta t_k$ | $5\times10^{-13}$ (rad/s)² |
| 角度噪声增量 | 姿态误差增量方差 | $N^2\Delta t_k$ | $2\times10^{-10}$ rad² |

（Python 复核 $\sigma_g^2=8\times10^{-6}$，$Q_b=5\times10^{-13}$。）

### 9.2 注意点

1. **数据手册 → 滤波器**要用同一套单位：°/√h、rad/√s、(rad/s)/√Hz 之间的换算（如 $1^\circ/\sqrt{\text{h}}=\frac{\pi}{180\cdot60}\ \text{rad}/\sqrt{\text{s}}=2.909\times10^{-4}$ rad/√s）；
2. **实机常需放大**：静置标定得到的噪声通常比实机运行时小——振动、温度变化、电机电磁干扰、量化、未建模的动力学都会让实际误差更大。工程上常把 Allan 得到的 $Q$、$R$ 作为**下限**，再按实机新息一致性（NIS）向上调整；
3. **采样/滤波周期变化**时，$Q$、$R$ 必须按 $\Delta t$ 重算；
4. **有色成分**：零偏不稳定性（平底）不是标准的白噪声或随机游走，常用一阶高斯—马尔可夫或把 $Q_b$ 取得比 RRW 理论值大一些来覆盖。

## 10. 统计置信度与注意事项

- **数据长度**：Allan 曲线上每个 $\tau$ 的可信度取决于簇数，估计 $\tau$ 处的偏差至少需要 $10\tau$ 到 $20\tau$ 的数据；
- **温度**：IMU 零偏与温度强相关，静置数据要在稳定温度下采集，或补偿温度后再算；
- **多次实验**：对同一块芯片重复采集多条曲线，取平均；对不同芯片取分布，才能估计个体差异；
- **采集条件**：静置在无振动的平台上，磁、电干扰尽量小，供电稳定；
- **与 PSD 的互补**：Allan 方差擅长长时间尺度（慢漂移）；Welch PSD 擅长中高频（谐波、共振）。同一份数据两种分析都做，才能覆盖所有时间尺度。

一个可操作的小流程，把本章的几块拼起来：（1）静置采集至少 1–2 小时的原始数据，记录温度；（2）计算重叠 Allan 偏差，画双对数图，读出 $N$（$\tau=1$ s）、曲线最低点与零偏不稳定性 $B$、长 $\tau$ 端的 $K$；（3）用 Welch PSD 检查是否存在窄带干扰（电源频率、传感器内部采样的混叠线），若有，则先滤除再算 Allan；（4）按第 9 节把参数换算为 $R$、$Q$ 的初值；（5）上实机运行，记录新息，检查其自相关与 NIS，必要时只放大 $Q$ 而不改 $R$（或相反），一次只动一个量；（6）把最终取值与依据写进调参记录，这样换传感器或改采样率时可以有章可循。这套流程的价值在于：每个数字都有来源，面试时可以回答"为什么取这个值"，而不是"调出来的"。

## 11. 在本项目里怎么用

> 说明：CV 没有写 Allan 方差标定；这里讲的是"噪声参数从哪儿来、为什么这样取"。面试里说成"我对 IMU 噪声的理解与取值依据"，不要说成"我做了 Allan 标定"，除非你确实做过。

- **P1 / P3 / P7 姿态估计**：互补滤波（Mahony）里的比例/积分增益的物理含义，可以用陀螺的零偏漂移速度和加速度计噪声的时间尺度来解释（见 [互补滤波与 Mahony AHRS](../06_状态估计/02_互补滤波与Mahony_AHRS.md)）：积分增益要能跟上零偏的漂移，但又不能把噪声当零偏。
- **P5 EKF**：IMU 噪声密度、轮速噪声如何进入 $Q$、$R$，以及"为什么实际取值比数据手册的更大"，是面试高频追问（见 [卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md) 的 Q、R 调参一节和 [里程计与传感器融合](../06_状态估计/04_里程计与传感器融合.md)）。
- **P7 状态估计**：轮速、关节编码器差分得到的速度，其噪声主要是量化噪声（白噪声的一种），按 $\sigma_d$ 与采样周期的关系估计（见 [P7 状态估计与速度、位置](../../01_项目课程/P7_可变腿长轮腿机器人/08_状态估计与速度_位置.md)）。
- **P6 状态一致性**：多源数据的"可信度"（协方差）如何标注，是把噪声模型放进状态管理接口的依据（见 [机器人状态管理](../../01_项目课程/P6_深开鸿M-Robots平台软件/05_机器人状态管理.md)）。

## 12. 常见错误与故障现象

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 换采样率后滤波效果变差 | $R$ 仍用旧采样率下的方差 | 由 $q$ 重新换算：$\sigma_d^2=q/\Delta t$ |
| 滤波器过于自信，轨迹漂移但协方差很小 | 有色/相关噪声被当白噪声 | 扩维建模，或增大 $R$ |
| 数据手册换算后 $R$ 偏出 2–6 倍 | 单边/双边、Hz/(rad/s) 混淆 | 严格核对 ASD 的定义和单位 |
| 姿态长期漂移，短期很稳 | 零偏随机游走/温漂 | 估计零偏作为状态；温度补偿 |
| Allan 曲线右端剧烈抖动 | 长 $\tau$ 簇数太少 | 采更长数据，只信前 1/10 |
| 静置数据看不到零偏不稳定性平底 | 噪声大、采样不够长，或温度变化 | 稳温、延长记录 |
| 新息自相关显著非零 | 模型漏掉动态/有色噪声/延迟 | 补建模，查时间同步 |
| 直接对原始速率求平均降噪后仍漂 | 平均不能降低相关误差 | 区分白噪声与漂移，分别处理 |

## 13. 面试怎么问、怎么答

**Q1. What is Allan variance and what do you read from it?**

> Allan variance compares the averages of adjacent clusters of stationary data for different averaging times. On a log–log plot, a slope of −1/2 at short times is white rate noise, which gives the angle random walk at one second. A flat floor is bias instability, and a slope of +1/2 at long times is rate random walk. It separates noise sources by time scale, which a single standard deviation cannot do.

中文：Allan 方差比较不同平均时间下相邻簇平均值的差；双对数图中短时间斜率 −1/2 是速率白噪声，在 1 s 处读出 ARW；平底是零偏不稳定性；长时间斜率 +1/2 是速率随机游走；它按时间尺度分离噪声源，单一标准差做不到。追问：最低点代表什么？（白噪声与随机游走影响相等的时间尺度，也是该平均时间下的最佳零偏估计精度。）

**Q2. How do you get the Kalman filter noise parameters from a gyro data sheet?**

> The angle random walk, in rad per root second, is the square root of the continuous white-noise intensity. The discrete measurement variance is N squared divided by the sample time, so it changes with the rate. The bias random walk coefficient K gives the bias process noise as K squared times the time step. I treat these as lower bounds and then tune upward against the filter's innovation statistics on the real robot, because vibration and temperature make the real noise larger.

中文：ARW（rad/√s）是连续白噪声强度的平方根；离散方差 $=N^2/\Delta t$，随采样率变；零偏随机游走系数 $K$ 给出零偏过程噪声 $K^2\Delta t$；这些是下限，再用实机新息统计向上调，因为振动与温度使真实噪声更大。

**Q3. Why does the standard deviation of a sample get larger when I sample faster?**

> Because each sample averages the signal over a shorter window. For white noise of intensity q, the variance of a sample is q divided by the sample period. The information per unit time does not change: if I average the faster samples back over a second, I get the same noise as before.

中文：采样更快意味着每个采样平均的窗口更短，白噪声强度 $q$ 时每个采样方差为 $q/\Delta t$；单位时间的信息量不变，把快采样平均回 1 s 的噪声与之前相同。

**Q4. What is a random walk and why is it dangerous for dead reckoning?**

> A random walk is the integral of white noise. Its variance grows linearly with time, so its standard deviation grows with the square root of time and never settles. If the gyro bias itself does a random walk, the angle error grows like time to the power 1.5. That is why integrating the gyro alone drifts without bound, and why the bias must be estimated using an absolute reference such as gravity or a magnetometer.

中文：随机游走是白噪声的积分，方差随时间线性增长、标准差随 $\sqrt t$ 增长、不收敛；零偏本身随机游走时角度误差按 $t^{1.5}$ 增长；所以单独积分陀螺必然无界漂移，必须用重力或磁力计等绝对参考估计零偏。

**Q5. How do you check whether your filter's noise model is consistent?**

> I look at the innovation sequence. For a matched model it is zero-mean, white, and its covariance matches the predicted innovation covariance. I check the sample autocorrelation against the plus or minus two over root N bound, and the normalised innovation squared against a chi-square interval. Significant correlation means that something, a bias, a delay or coloured noise, is missing from the model.

中文：看新息序列；模型匹配时应零均值、白、协方差与预测一致；用样本自相关对照 $\pm2/\sqrt N$ 界，用 NIS 对照卡方区间；相关显著说明漏了偏置、延迟或有色噪声。

## 14. 本章要点速记

1. 平稳过程：均值恒定、自相关只依赖 $\tau$；$R(0)=\sigma^2$；Wiener–Khinchin：自相关与 PSD 互为傅里叶变换。
2. 连续白噪声强度 $q$ 对应离散方差 $\sigma_d^2=q/\Delta t$；采样越快，每个采样越"吵"。
3. $N$（ARW）=$\sqrt q$；$\sigma_d=N/\sqrt{\Delta t}$；角度误差 $N\sqrt t$。
4. 一阶高斯—马尔可夫：$R=\sigma^2e^{-|\tau|/T}$；离散 $\phi=e^{-\Delta t/T}$，$r[k]=\phi^{|k|}$。
5. 随机游走方差线性增长；零偏随机游走使角度误差按 $Kt^{3/2}/\sqrt3$ 增长。
6. 白性检验：样本自相关应在 $\pm2/\sqrt N$ 内。
7. Allan 偏差：ARW 斜率 −1/2（1 s 读 $N$）；零偏不稳定性平底（$0.664B$）；RRW 斜率 +1/2。
8. $\sigma_A^2=N^2/\tau+K^2\tau/3$，最低点 $\tau^\ast=\sqrt3N/K$。
9. 滤波器：$R_g=N^2/\Delta t_k$，$Q_b=K^2\Delta t_k$；这些是下限，实机要向上调。
10. 估计 Allan 偏差的相对误差约 $1/\sqrt{2(M-1)}$，长 $\tau$ 需要长数据。

## 15. 自测题

1. 采样 200 Hz，ARW $N=3\times10^{-4}$ rad/√s，离散测量标准差？（$3\times10^{-4}/\sqrt{0.005}=4.24\times10^{-3}$ rad/s。）
2. $T=2$ s，$\Delta t=0.05$ s，AR(1) 的 $\phi$ 与 $r[40]$？（$\phi=e^{-0.025}=0.9753$；$r[40]=e^{-1}=0.368$。）
3. 白噪声方差 $\sigma_d^2$ 的采样序列，平均 100 个点后方差是多少？（$\sigma_d^2/100$。）
4. Allan 曲线在 $\tau=1$ s 处读到 $1.5\times10^{-4}$，且此处斜率为 −1/2，ARW 是多少？（$N\approx1.5\times10^{-4}$ rad/√s。）
5. 1000 个残差样本，样本自相关 $\hat r[3]=0.09$，是否通过白性检验？（界为 $\pm0.063$，$0.09$ 越界，不通过。）
6. 为什么 Allan 曲线右端不可靠？（簇数少，估计相对误差 $\sim1/\sqrt{2(M-1)}$ 大。）

## 16. 相关链接

- 上一章：[频谱分析：FFT、窗函数、PSD 与振动诊断](./03_频谱分析_FFT_窗函数_PSD与振动诊断.md)；模块首章：[信号与系统基础](./01_信号与系统基础_卷积_傅里叶_拉普拉斯_Z变换.md)
- 相关基础：[传感器：IMU、编码器、激光雷达](../06_状态估计/01_传感器_IMU_编码器_激光雷达.md)、[卡尔曼滤波与 EKF](../06_状态估计/03_卡尔曼滤波与EKF.md)、[概率统计与高斯分布](../01_数学基础/03_概率统计与高斯分布.md)
- 项目：[P1](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)、[P5](../../01_项目课程/P5_车式移动机器人自主导航/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
- 模块说明：[04_信号与系统 README](./README.md)
